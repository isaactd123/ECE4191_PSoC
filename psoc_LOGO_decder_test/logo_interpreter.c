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

/* ================== HOOK: VOWEL DETECTION ================== */

/*
    At the end of every while-loop iteration the turtle
    should check whether a vowel is under it and update :vowel.

    This is a STUB. By default it always reports "blank", which means
    a while loop keyed on :vowel = "blank will run forever until you
    fill this in. Replace the body with your actual decision-tree
    classification, then call setVariable("vowel", ...) with the
    result: "a, "e, "i, "o, "u, or "blank.

    LOGO_BENCH_TEST below is a temporary shortcut so you can watch the
    WHOLE interpreter run (including the while loop actually exiting
    and the ifelse branch firing) without any real sensors wired up.
    Set it to 0 once your real vowel classification is ready.
*/
#define LOGO_BENCH_TEST 1

static void updateVowelFromSensors(void)
{
#if LOGO_BENCH_TEST
    static int callCount = 0;
    Value v;

    callCount++;
    v.type = VAL_WORD;

    if (callCount >= 3)
    {
        /* Pretend a vowel 'a' was found after 3 loop iterations,
           so the while loop exits and ifelse gets exercised. */
        strcpy(v.word, "a");
    }
    else
    {
        strcpy(v.word, "blank");
    }

    setVariable("vowel", v);
#else
    Value blank;
    blank.type = VAL_WORD;
    strcpy(blank.word, "blank");
    setVariable("vowel", blank);
#endif
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

    for (;;)
    {
        int conditionTrue;
        int blockStart;

        position = conditionStart;
        conditionTrue = parseCondition();
        if (interpreterError) return;

        blockStart = position; /* now sitting at the block's '[' */

        if (conditionTrue)
        {
            position = blockStart;
            executeBlock();
            if (interpreterError) return;

            updateVowelFromSensors(); /* re-check for a vowel at the new square */
            /* loop back around and re-check the condition */
        }
        else
        {
            position = blockStart;
            skipBlock();
            return; /* while statement is done */
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
        if (!interpreterError) motorForward(n);
    }
    else if (strcmp(command, "bk") == 0)
    {
        int n = parseNumericValue();
        if (!interpreterError) motorBackward(n);
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