/* ========================================
 *
 * LOGO interpreter for the WriteCodeBot turtle (PSoC 5LP)
 *
 * Supports: make, while, ifelse, repeat, fd, bk, lt, rt
 *
 * ========================================
*/
#include "project.h"
#include "logo_interpreter.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

/* ================== CONFIGURATION ================== */

#define MAX_TOKENS      220   /* generous headroom for a 240-byte program   */
#define MAX_VARIABLES   8     /* the program only ever needs vowel+checkpoint */
#define MAX_WORD_LEN    20    /* longest identifier / word / variable name   */

/*
    Calibration constants for the placeholder motor functions below.
    These are open-loop timing values (no encoder feedback). You WILL
    need to tune these on your actual robot: send a program with a
    single "fd 1" and adjust MS_PER_SQUARE until it moves exactly one
    100 mm square. Same idea for MS_PER_45_DEGREES with "rt 45".
*/
#define MS_PER_SQUARE      700
#define MS_PER_45_DEGREES  300

/*
    Time to wait after the motors stop before the camera captures.
    The chassis keeps rocking briefly after a move and a blurred
    frame classifies wrongly. Tune this on the real robot.
*/
#define SETTLE_AFTER_MOVE_MS 400

/* How long to wait for the ESP32 to answer a CLASSIFY request. */
#define VOWEL_RESPONSE_TIMEOUT_MS 3000

/* Refuse to spin forever if :vowel never changes. */
#define MAX_WHILE_ITERATIONS 200

/* ================== VALUES ================== */

/*
    A LOGO value is either a "word (string, e.g. "blank, "a) or a
    NUMBER. Variables hold one of these, and conditions compare two
    of these.
*/
typedef enum
{
    VAL_WORD,
    VAL_NUMBER
} ValueType;

typedef struct
{
    ValueType type;
    char word[MAX_WORD_LEN];
    int number;
} Value;

/* ================== VARIABLE TABLE ================== */

typedef struct
{
    char name[MAX_WORD_LEN];
    Value value;
} Variable;

static Variable variables[MAX_VARIABLES];
static int variableCount = 0;

static Value *findVariable(const char *name)
{
    int idx;
    for (idx = 0; idx < variableCount; idx++)
    {
        if (strcmp(variables[idx].name, name) == 0)
        {
            return &variables[idx].value;
        }
    }
    return NULL;
}

static void setVariable(const char *name, Value value);  /* forward declare, used below and by reportError paths */

/* ================== TOKENS ================== */

typedef enum
{
    TOK_IDENTIFIER,   /* make, while, ifelse, repeat, fd, bk, lt, rt */
    TOK_WORD,         /* "vowel, "blank, "a, ...                     */
    TOK_VARIABLE,     /* :vowel, :checkpoint, ...                    */
    TOK_NUMBER,       /* 4, 135, 2, ...                              */
    TOK_LBRACKET,     /* [                                           */
    TOK_RBRACKET,     /* ]                                           */
    TOK_EQUALS,       /* =                                           */
    TOK_EOF
} TokenKind;

typedef struct
{
    TokenKind kind;
    char text[MAX_WORD_LEN];
} Token;

static Token tokens[MAX_TOKENS];
static int tokenCount = 0;
static int position = 0;

/* ================== ERROR HANDLING ================== */

static int interpreterError = 0;

/*
    Counts completed classifications. parseWhile() uses it to detect
    a loop body that never moves the turtle, which would otherwise
    spin forever without ever refreshing :vowel.
*/
static int classificationCount = 0;

static void reportError(const char *message)
{
    /* Only report the FIRST error -- once execution has already
       stopped, extra messages just clutter Termite. */
    if (interpreterError) return;

    interpreterError = 1;
    UART_1_PutString("[LOGO ERROR] ");
    UART_1_PutString(message);
    UART_1_PutString("\r\n");
}

/* ================== TOKENIZER ================== */

static void addToken(TokenKind kind, const char *start, int length)
{
    if (tokenCount >= MAX_TOKENS - 1) /* leave room for the EOF token */
    {
        reportError("Program produced too many tokens");
        return;
    }

    if (length >= MAX_WORD_LEN) length = MAX_WORD_LEN - 1;

    if (length > 0)
    {
        memcpy(tokens[tokenCount].text, start, length);
    }
    tokens[tokenCount].text[length] = '\0';
    tokens[tokenCount].kind = kind;
    tokenCount++;
}

static void tokenize(const char *program)
{
    int index = 0;
    int length = (int)strlen(program);

    tokenCount = 0;

    while (index < length)
    {
        char c = program[index];

        /* Whitespace (spaces, tabs, CR, LF) just separates tokens */
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n')
        {
            index++;
            continue;
        }

        if (c == '[')
        {
            addToken(TOK_LBRACKET, NULL, 0);
            index++;
            continue;
        }

        if (c == ']')
        {
            addToken(TOK_RBRACKET, NULL, 0);
            index++;
            continue;
        }

        if (c == '=')
        {
            addToken(TOK_EQUALS, NULL, 0);
            index++;
            continue;
        }

        /* "word or :variable -- skip the prefix, read the name */
        if (c == '"' || c == ':')
        {
            TokenKind kind = (c == '"') ? TOK_WORD : TOK_VARIABLE;
            int start;

            index++; /* skip the " or : */
            start = index;

            while (index < length &&
                   (isalnum((unsigned char)program[index]) || program[index] == '_'))
            {
                index++;
            }

            addToken(kind, &program[start], index - start);
            continue;
        }

        /* A number, optionally with a leading + or - */
        if (isdigit((unsigned char)c) ||
            ((c == '+' || c == '-') && index + 1 < length &&
             isdigit((unsigned char)program[index + 1])))
        {
            int start = index;
            index++;
            while (index < length && isdigit((unsigned char)program[index])) index++;

            addToken(TOK_NUMBER, &program[start], index - start);
            continue;
        }

        /* A bare word: command names like fd, while, repeat, ... */
        if (isalpha((unsigned char)c) || c == '_')
        {
            int start = index;
            while (index < length &&
                   (isalnum((unsigned char)program[index]) || program[index] == '_'))
            {
                index++;
            }

            addToken(TOK_IDENTIFIER, &program[start], index - start);
            continue;
        }

        /* Anything else is not part of this LOGO grammar */
        reportError("Unexpected character in program");
        index++;
    }

    addToken(TOK_EOF, NULL, 0);
}

/* ================== TOKEN STREAM HELPERS ================== */

static Token *currentToken(void)
{
    return &tokens[position];
}

static void advanceToken(void)
{
    if (tokens[position].kind != TOK_EOF) position++;
}

/* ================== VARIABLES (definition) ================== */

static void setVariable(const char *name, Value value)
{
    Value *existing = findVariable(name);

    if (existing != NULL)
    {
        *existing = value;
        return;
    }

    if (variableCount < MAX_VARIABLES)
    {
        strncpy(variables[variableCount].name, name, MAX_WORD_LEN - 1);
        variables[variableCount].name[MAX_WORD_LEN - 1] = '\0';
        variables[variableCount].value = value;
        variableCount++;
    }
    else
    {
        reportError("Too many variables (increase MAX_VARIABLES)");
    }
}

/* ================== VALUES & CONDITIONS ================== */

static Value parseValue(void)
{
    Value result;
    Token *tok = currentToken();

    if (tok->kind == TOK_WORD)
    {
        result.type = VAL_WORD;
        strncpy(result.word, tok->text, MAX_WORD_LEN - 1);
        result.word[MAX_WORD_LEN - 1] = '\0';
        advanceToken();
        return result;
    }

    if (tok->kind == TOK_NUMBER)
    {
        result.type = VAL_NUMBER;
        result.number = atoi(tok->text);
        advanceToken();
        return result;
    }

    if (tok->kind == TOK_VARIABLE)
    {
        Value *stored = findVariable(tok->text);
        advanceToken();

        if (stored != NULL)
        {
            return *stored;
        }

        reportError("Unknown variable referenced");
        result.type = VAL_WORD;
        result.word[0] = '\0';
        return result;
    }

    reportError("Expected a word, number, or variable");
    result.type = VAL_WORD;
    result.word[0] = '\0';
    return result;
}

static int parseNumericValue(void)
{
    Value v = parseValue();

    if (v.type == VAL_NUMBER) return v.number;

    reportError("Expected a number");
    return 0;
}

static int valuesEqual(Value a, Value b)
{
    if (a.type == VAL_NUMBER && b.type == VAL_NUMBER)
    {
        return a.number == b.number;
    }

    if (a.type == VAL_WORD && b.type == VAL_WORD)
    {
        return strcmp(a.word, b.word) == 0;
    }

    return 0; /* a word can never equal a number */
}

/* Reads "value = value" and returns whether it's true right now.
   Used by both while and ifelse. */
static int parseCondition(void)
{
    Value left = parseValue();

    if (currentToken()->kind != TOK_EQUALS)
    {
        reportError("Expected '=' in condition");
        return 0;
    }
    advanceToken();

    Value right = parseValue();

    return valuesEqual(left, right);
}

/* ================== VOWEL DETECTION VIA THE ESP32 ================== */

/*
    The camera and the classifier live on the ESP32-S3. The PSoC asks
    for a reading and waits for the answer:

        PSoC  -> UART_2:  CLASSIFY\n
        ESP32 -> UART_2:  VOWEL:a\n      (or VOWEL:blank\n)

    UART_1 stays free for Termite debug output.
*/

/*
    Reads one newline-terminated line from UART_2 into 'line'.
    Returns 1 on success, 0 if the timeout expired first.
*/
static int readLineFromEsp(char *line, int capacity)
{
    int length = 0;
    uint32 elapsed = 0u;

    while (elapsed < VOWEL_RESPONSE_TIMEOUT_MS)
    {
        if (UART_2_GetRxBufferSize() > 0u)
        {
            char c = (char)UART_2_GetChar();

            if (c == '\n' || c == '\r')
            {
                if (length > 0)
                {
                    line[length] = '\0';
                    return 1;
                }
                continue;   /* ignore blank lines */
            }

            if (length < capacity - 1)
            {
                line[length] = c;
                length++;
            }
        }
        else
        {
            CyDelay(1);
            elapsed++;
        }
    }

    line[0] = '\0';
    return 0;
}

static void updateVowelFromSensors(void)
{
    char line[32];
    Value v;

    v.type = VAL_WORD;

    /* Ask the ESP32 to classify the square underneath the turtle. */
    UART_2_PutString("CLASSIFY\n");

    if (!readLineFromEsp(line, (int)sizeof(line)))
    {
        UART_1_PutString("[LOGO] Classifier timed out, assuming blank\r\n");
        strcpy(v.word, "blank");
        setVariable("vowel", v);
        return;
    }

    UART_1_PutString("[LOGO] Classifier replied: ");
    UART_1_PutString(line);
    UART_1_PutString("\r\n");

    if (strncmp(line, "VOWEL:", 6) != 0)
    {
        /*
            Not a classification line. The ESP32 also prints status
            text, so anything unrecognised is treated as blank rather
            than as an error.
        */
        strcpy(v.word, "blank");
        setVariable("vowel", v);
        return;
    }

    if (strcmp(line + 6, "blank") == 0)
    {
        strcpy(v.word, "blank");
    }
    else
    {
        /* Single character: a, e, i, o or u. */
        v.word[0] = line[6];
        v.word[1] = '\0';
    }

    setVariable("vowel", v);
}

/*
    Called once each time the turtle finishes a move and comes to
    rest on a new square. Turns do not change which square the turtle
    occupies, so lt and rt deliberately do not call this.
*/
static void arriveAtNewSquare(void)
{
    if (interpreterError) return;

    /* Let the chassis stop moving before the camera captures. */
    CyDelay(SETTLE_AFTER_MOVE_MS);

    UART_1_PutString("[LOGO] Arrived. Classifying...\r\n");

    updateVowelFromSensors();
    classificationCount++;
}

/* ================== MOTOR PRIMITIVES (PLACEHOLDERS) ================== */

/*
    These four functions are where LOGO commands become physical
    movement. Right now they just print what they WOULD do, so you
    can verify the whole interpreter over Termite before any motors
    are wired up.

    Once you tell me your motor driver setup (H-bridge? stepper?
    servo? PWM component names in TopDesign?) replace the bodies
    below with real motor control -- the timing constants at the
    top of this file (MS_PER_SQUARE, MS_PER_45_DEGREES) are there
    for exactly that.
*/

static void printAction(const char *action, int amount, const char *unit)
{
    char numText[8];
    sprintf(numText, "%d", amount);

    UART_1_PutString("[ACTION] ");
    UART_1_PutString(action);
    UART_1_PutString(" ");
    UART_1_PutString(numText);
    UART_1_PutString(" ");
    UART_1_PutString(unit);
    UART_1_PutString("\r\n");
}

static void motorForward(int squares)
{
    printAction("Forward", squares, "square(s)");

    /* TODO: replace with real motor control, e.g.:
         PWM_Left_WriteCompare(FORWARD_DUTY);
         PWM_Right_WriteCompare(FORWARD_DUTY);
         CyDelay(MS_PER_SQUARE * squares);
         PWM_Left_WriteCompare(0);
         PWM_Right_WriteCompare(0);
    */
}

static void motorBackward(int squares)
{
    printAction("Backward", squares, "square(s)");
    /* TODO: same idea as motorForward(), reversed */
}

static void motorTurnLeft(int degrees)
{
    printAction("Turn left", degrees, "degrees");
    /* TODO: spin wheels in opposite directions for
       (degrees / 45) * MS_PER_45_DEGREES milliseconds */
}

static void motorTurnRight(int degrees)
{
    printAction("Turn right", degrees, "degrees");
    /* TODO: mirror of motorTurnLeft() */
}

/* ================== BLOCK HANDLING ================== */

/* Forward declarations -- these functions call each other
   (a block contains statements, and a statement can contain blocks) */
static void parseStatement(void);
static void executeBlock(void);
static void skipBlock(void);

/*
    Skips over a [ ... ] block WITHOUT running it. Correctly handles
    blocks nested inside blocks (e.g. repeat 2 [fd 5] sitting inside
    an ifelse block) by counting bracket depth instead of just
    looking for the next ].
*/
static void skipBlock(void)
{
    int depth;

    if (currentToken()->kind != TOK_LBRACKET)
    {
        reportError("Expected '[' to start a block");
        return;
    }

    depth = 0;
    do
    {
        if (currentToken()->kind == TOK_LBRACKET) depth++;
        if (currentToken()->kind == TOK_RBRACKET) depth--;

        if (currentToken()->kind == TOK_EOF)
        {
            reportError("Missing ']' for a block");
            return;
        }

        advanceToken();
    }
    while (depth > 0);
}

/* Runs every statement inside a [ ... ] block, once. */
static void executeBlock(void)
{
    if (currentToken()->kind != TOK_LBRACKET)
    {
        reportError("Expected '[' to start a block");
        return;
    }
    advanceToken(); /* consume '[' */

    while (currentToken()->kind != TOK_RBRACKET && !interpreterError)
    {
        if (currentToken()->kind == TOK_EOF)
        {
            reportError("Missing ']' for a block");
            return;
        }
        parseStatement();
    }

    advanceToken(); /* consume ']' */
}

/* ================== STATEMENTS ================== */

static void parseMake(void)
{
    Token *nameToken = currentToken();
    char varName[MAX_WORD_LEN];
    Value v;

    if (nameToken->kind != TOK_WORD)
    {
        reportError("Expected a variable name after 'make'");
        return;
    }

    strncpy(varName, nameToken->text, MAX_WORD_LEN - 1);
    varName[MAX_WORD_LEN - 1] = '\0';
    advanceToken();

    v = parseValue();
    if (interpreterError) return;

    setVariable(varName, v);
}

/*
    The key trick for loops: because tokens live in an array (not a
    one-way stream), we can save "position" before the condition,
    then reset position back to that saved value to re-check the
    condition as many times as needed.
*/
static void parseWhile(void)
{
    int conditionStart = position;
    int iterations = 0;

    for (;;)
    {
        int conditionTrue;
        int blockStart;
        int countBeforeBlock;

        position = conditionStart;
        conditionTrue = parseCondition();
        if (interpreterError) return;

        blockStart = position; /* now sitting at the block's '[' */

        if (!conditionTrue)
        {
            position = blockStart;
            skipBlock();
            return; /* while statement is done */
        }

        if (++iterations > MAX_WHILE_ITERATIONS)
        {
            reportError("while loop exceeded the iteration limit");
            return;
        }

        countBeforeBlock = classificationCount;

        position = blockStart;
        executeBlock();
        if (interpreterError) return;

        /*
            If the body moved the turtle, arriveAtNewSquare() has
            already refreshed :vowel. If it did not move at all --
            for example while :vowel = "blank [rt 45] -- classify
            here so the condition can still change.
        */
        if (classificationCount == countBeforeBlock)
        {
            updateVowelFromSensors();
            classificationCount++;
        }
    }
}

static void parseIfElse(void)
{
    int conditionTrue = parseCondition();
    int block1Start;

    if (interpreterError) return;

    block1Start = position;

    if (conditionTrue)
    {
        position = block1Start;
        executeBlock();
        if (interpreterError) return;

        skipBlock(); /* skip the else-block, don't run it */
    }
    else
    {
        position = block1Start;
        skipBlock(); /* skip the if-block, don't run it */

        executeBlock(); /* now run the else-block */
    }
}

static void parseRepeat(void)
{
    int count = parseNumericValue();
    int blockStart;
    int iteration;

    if (interpreterError) return;

    blockStart = position;

    if (count <= 0)
    {
        position = blockStart;
        skipBlock();
        return;
    }

    for (iteration = 0; iteration < count; iteration++)
    {
        position = blockStart;
        executeBlock();
        if (interpreterError) return;
    }
}

static void parseStatement(void)
{
    Token *cmdToken = currentToken();
    char command[MAX_WORD_LEN];

    if (cmdToken->kind != TOK_IDENTIFIER)
    {
        reportError("Expected a command");
        return;
    }

    strncpy(command, cmdToken->text, MAX_WORD_LEN - 1);
    command[MAX_WORD_LEN - 1] = '\0';
    advanceToken();

    if (strcmp(command, "make") == 0)
    {
        parseMake();
    }
    else if (strcmp(command, "while") == 0)
    {
        parseWhile();
    }
    else if (strcmp(command, "ifelse") == 0)
    {
        parseIfElse();
    }
    else if (strcmp(command, "repeat") == 0)
    {
        parseRepeat();
    }
    else if (strcmp(command, "fd") == 0)
    {
        int n = parseNumericValue();
        if (!interpreterError)
        {
            motorForward(n);
            arriveAtNewSquare();
        }
    }
    else if (strcmp(command, "bk") == 0)
    {
        int n = parseNumericValue();
        if (!interpreterError)
        {
            motorBackward(n);
            arriveAtNewSquare();
        }
    }
    else if (strcmp(command, "lt") == 0)
    {
        int n = parseNumericValue();
        if (!interpreterError) motorTurnLeft(n);
    }
    else if (strcmp(command, "rt") == 0)
    {
        int n = parseNumericValue();
        if (!interpreterError) motorTurnRight(n);
    }
    else
    {
        reportError("Unknown command");
    }
}

/* ================== PUBLIC ENTRY POINT ================== */

void runLogoProgram(const char *program)
{
    interpreterError = 0;
    variableCount = 0;
    position = 0;
    classificationCount = 0;

    tokenize(program);
    if (interpreterError) return;

    UART_1_PutString("[LOGO] Starting program execution...\r\n");

    while (currentToken()->kind != TOK_EOF && !interpreterError)
    {
        parseStatement();
    }

    if (!interpreterError)
    {
        UART_1_PutString("[LOGO] Program finished.\r\n");
    }
}