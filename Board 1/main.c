/*=============================================================================
    Battle Arena : Multiplayer Fighting Game over UART
    Target      : Nuvoton NUC140VE3AN (NuTiny / LB-002 Learning Board)
    Toolchain   : Keil uVision (ARM-ADS, armcc)

    Flash this SAME .c file to BOTH boards - it works for either player.
    Connect the two boards' UART0 TX/RX/GND together (cross TX<->RX).

    -------------------------------------------------------------------------
    !! EDIT THIS PER BOARD BEFORE FLASHING !!
    -------------------------------------------------------------------------
    Set I_AM_BOY to 1 on one board and 0 on the other, just below. That's
    the ONLY difference between the two boards' firmware - it selects which
    character sprite you play as and which physical keys (of the 6 on the
    keypad) move/attack with.

    Key codes and the boy/girl sprite set are taken from your working
    2-player local demo (ScanKey() returns 1-6 for the 6 keypad buttons,
    0/other = idle):
        keys 1,2,3 = girl: move-left, attack, move-right
        keys 4,5,6 = boy : move-left, attack, move-right
    Each board only plays ONE character, so the 3 unused keys from the
    other character's set are repurposed here for kick/special/block.
=============================================================================*/

#include <stdio.h>
#include "NUC1xx.h"
#include "SYS.h"
#include "GPIO.h"
#include "UART.h"
#include "LCD.h"
#include "Scankey.h"
#include "Seven_Segment.h"
#include "Sprites.h"

#define I_AM_BOY   1     /* TODO: flip to 0 on the second board */

/*-----------------------------------------------------------------------
  Board / pin configuration  (TODO: confirm against your LB-002 wiring)
-----------------------------------------------------------------------*/
#define UART_CH             UART_PORT0
#define UART_BAUD           115200

#define LED_PORT            E_GPB
#define LED_GREEN_PIN       0          /* HP > 70                    */
#define LED_YELLOW_PIN      1          /* 30 <= HP <= 70              */
#define LED_RED_PIN         2          /* HP < 30 (blinks)            */

#define BUZZER_PORT         E_GPB
#define BUZZER_PIN          3

/*-----------------------------------------------------------------------
  Keypad mapping - verified against the working reference demo.
  KEY_NONE = 0 (ScanKey() never matches any of the switch cases 1-6 when
  idle in that demo, so 0/anything-else is treated as "no key").
-----------------------------------------------------------------------*/
#define KEY_NONE            0

#if I_AM_BOY
    #define KEY_LEFT        4
    #define KEY_PUNCH       5
    #define KEY_RIGHT       6
    #define KEY_KICK        1
    #define KEY_SPECIAL     2
    #define KEY_BLOCK       3
#else
    #define KEY_LEFT        1
    #define KEY_PUNCH       2
    #define KEY_RIGHT       3
    #define KEY_KICK        4
    #define KEY_SPECIAL     5
    #define KEY_BLOCK       6
#endif

/*-----------------------------------------------------------------------
  Character sprite selection
-----------------------------------------------------------------------*/
#if I_AM_BOY
    #define MY_SPRITE       boy
    #define MY_DEFEAT       boydefeat
    #define MY_WIN          boywin
    #define ENEMY_SPRITE    girl
    #define ENEMY_DEFEAT    girldefeat
#else
    #define MY_SPRITE       girl
    #define MY_DEFEAT       girldefeat
    #define MY_WIN          girlwin
    #define ENEMY_SPRITE    boy
    #define ENEMY_DEFEAT    boydefeat
#endif

#define SPRITE_Y            16   /* sprites are 32x48, screen is 128x64 -
                                     y=16..64 is exactly the sprite band */
#define SPRITE_MIN_X        1
#define SPRITE_MAX_X        95
#define MOVE_STEP           3
#define HIT_RANGE           30   /* max pixel gap for an attack to land */

/*-----------------------------------------------------------------------
  Game constants
-----------------------------------------------------------------------*/
#define START_HP            100
#define MATCH_SECONDS       60

#define DMG_PUNCH           5
#define DMG_KICK            10
#define DMG_SPECIAL         20

#define SPECIAL_COOLDOWN_S  10
#define BLOCK_COOLDOWN_S    5
#define BLOCK_ACTIVE_S      2

/*-----------------------------------------------------------------------
  UART packet protocol   [ CMD , DATA ]   (2 bytes per packet)
-----------------------------------------------------------------------*/
#define CMD_START           0x01
#define CMD_PUNCH           0x02
#define CMD_KICK            0x03
#define CMD_SPECIAL         0x04
#define CMD_BLOCK           0x05
#define CMD_HP_UPDATE       0x06
#define CMD_GAME_OVER       0x07
#define CMD_RESTART         0x08
#define CMD_POS_UPDATE      0x09   /* [CMD_POS_UPDATE, xPosition] - extra
                                       command beyond the original 8, needed
                                       to sync on-screen character position */

/*-----------------------------------------------------------------------
  Game state machine
-----------------------------------------------------------------------*/
typedef enum
{
    STATE_INTRO,
    STATE_WAIT_OPPONENT,
    STATE_COUNTDOWN,
    STATE_BATTLE,
    STATE_GAMEOVER
} E_GAME_STATE;

static E_GAME_STATE g_state;

static int16_t  g_myHP,    g_enemyHP, g_prevEnemyHP;
static int16_t  g_myX,     g_enemyX;
static uint16_t g_timeLeft;              /* seconds remaining          */
static uint8_t  g_specialCooldown;       /* seconds left before usable */
static uint8_t  g_blockCooldown;
static uint8_t  g_blockActiveTicks;      /* seconds the shield still holds */
static uint8_t  g_lastKey = KEY_NONE;    /* for simple debounce (edge detect) */

static char     g_lcdLine[24];

/*=============================================================================
    Low level helpers
=============================================================================*/

/* Blocking, but broken into small chunks so keypad/UART are polled during
   any "wait roughly N ms" pause instead of freezing the whole board.
   TODO: replace with a hardware Timer interrupt (Timer.c/h) for a
   production-quality non-blocking 1 Hz tick if you want it fully async. */
static void Delay_ms(uint32_t ms)
{
    DrvSYS_Delay(ms * 1000UL);
}

static void UART_SendPacket(uint8_t cmd, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = cmd;
    buf[1] = data;
    DrvUART_Write(UART_CH, buf, 2);
}

/* Non-blocking single-byte read. Uses the raw register macro directly
   instead of DrvUART_Read(), since DrvUART_Read() was observed reporting
   failure even when _DRVUART_RECEIVEAVAILABLE() shows bytes waiting. */
static int32_t UART_TryReceiveByte(uint8_t *outByte)
{
    if (_DRVUART_RECEIVEAVAILABLE(UART_CH) < 1)
        return 0;

    *outByte = (uint8_t)_DRVUART_RECEIVEBYTE(UART_CH);
    return 1;
}

/* Non-blocking receive: returns 1 and fills cmd/data if a full 2 byte
   packet is available, otherwise returns 0 immediately. */
static int32_t UART_TryReceivePacket(uint8_t *cmd, uint8_t *data)
{
    if (_DRVUART_RECEIVEAVAILABLE(UART_CH) < 2)
        return 0;

    if (!UART_TryReceiveByte(cmd))
        return 0;

    if (!UART_TryReceiveByte(data))
        return 0;

    return 1;
}

static void Buzzer_Beep(uint16_t on_ms, uint16_t off_ms, uint8_t repeat)
{
    uint8_t i;
    for (i = 0; i < repeat; i++)
    {
        DrvGPIO_SetBit(BUZZER_PORT, BUZZER_PIN);
        Delay_ms(on_ms);
        DrvGPIO_ClrBit(BUZZER_PORT, BUZZER_PIN);
        if (off_ms) Delay_ms(off_ms);
    }
}

static void LED_UpdateForHP(int16_t hp)
{
    if (hp <= 0)
    {
        DrvGPIO_SetBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_SetBit(LED_PORT, LED_YELLOW_PIN);
        DrvGPIO_SetBit(LED_PORT, LED_RED_PIN);
        return;
    }

    if (hp > 70)
    {
        DrvGPIO_SetBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_YELLOW_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);
    }
    else if (hp >= 30)
    {
        DrvGPIO_ClrBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_SetBit(LED_PORT, LED_YELLOW_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);
    }
    else
    {
        DrvGPIO_ClrBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_YELLOW_PIN);
        if ((g_timeLeft & 1) == 0)
            DrvGPIO_SetBit(LED_PORT, LED_RED_PIN);
        else
            DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);
    }
}

/* Matches the working pattern from Smpl_7seg's DisplayCounter(): each
   digit is only lit while its enable pin is held, so we must close then
   re-show each digit in turn. Must be called REPEATEDLY (many times a
   second) to stay visibly lit via persistence of vision. */
static void ShowTimer(uint16_t seconds)
{
    uint8_t tens = (uint8_t)(seconds / 10);
    uint8_t ones = (uint8_t)(seconds % 10);

    close_seven_segment();
    show_seven_segment(1, tens);
    DrvSYS_Delay(1200);

    close_seven_segment();
    show_seven_segment(0, ones);
    DrvSYS_Delay(1200);
}

/* Only redraws the single text status row (top of screen) - the sprite
   area below it (y=16..64) is drawn/erased separately by the sprite
   helpers below, only when something actually moves/changes, so this
   does NOT clear_LCD() the whole screen (that would wipe the sprites). */
static void RefreshStatusLine(void)
{
    sprintf(g_lcdLine, "P:%3d E:%3d T:%2d", g_myHP, g_enemyHP, g_timeLeft);
    print_Line(0, g_lcdLine);
}

/*=============================================================================
    Sprite helpers

    NOTE: sprite bitmaps are declared `const` in Sprites.c/h so the linker
    keeps them in flash instead of copying 14KB of image data into this
    chip's 16KB of RAM at startup. LCD.h's draw_Bmp32x48()/draw_LCD() take
    plain (non-const) unsigned char*, so we cast away const right here, in
    one place, when handing bitmaps to the vendor library - the library
    only reads these buffers, never writes them, so this is safe.
=============================================================================*/
static void DrawSprite(int16_t x, const unsigned char *bmp)
{
    draw_Bmp32x48(x, SPRITE_Y, FG_COLOR, BG_COLOR, (unsigned char *)bmp);
}

static void EraseSprite(int16_t x, const unsigned char *bmp)
{
    draw_Bmp32x48(x, SPRITE_Y, BG_COLOR, BG_COLOR, (unsigned char *)bmp);
}

/* Brief "got hit" animation: swap to the defeat bitmap for a moment, then
   back to normal. Used both when I get hit and (visually) when I land a
   confirmed hit on the enemy. */
static void FlashDefeat(int16_t x, const unsigned char *normalBmp, const unsigned char *defeatBmp)
{
    EraseSprite(x, normalBmp);
    DrawSprite(x, defeatBmp);
    Delay_ms(150);
    EraseSprite(x, defeatBmp);
    DrawSprite(x, normalBmp);
}

static uint8_t InAttackRange(void)
{
    int16_t d = g_myX - g_enemyX;
    if (d < 0) d = -d;
    return (d <= HIT_RANGE);
}

static void MoveMe(int16_t dx)
{
    EraseSprite(g_myX, MY_SPRITE);

    g_myX += dx;
    if (g_myX < SPRITE_MIN_X) g_myX = SPRITE_MIN_X;
    if (g_myX > SPRITE_MAX_X) g_myX = SPRITE_MAX_X;

    DrawSprite(g_myX, MY_SPRITE);
    UART_SendPacket(CMD_POS_UPDATE, (uint8_t)g_myX);
}

static void UpdateEnemyPos(uint8_t x)
{
    EraseSprite(g_enemyX, ENEMY_SPRITE);
    g_enemyX = x;
    DrawSprite(g_enemyX, ENEMY_SPRITE);
}

/*=============================================================================
    System bring-up
=============================================================================*/
static void Hardware_Init(void)
{
    STR_UART_T uartCfg;

    /* Clock/system init already happens automatically via SystemInit()
       (in system_NUC1xx.c), which the CMSIS startup code calls before
       main() runs - no manual SYS_Init() call needed/available here. */

    /* --- UART0 pin mux + open --- */
    DrvGPIO_InitFunction(E_FUNC_UART0_RX_TX);   /* TODO: change if using UART1/2 */

    uartCfg.u32BaudRate       = UART_BAUD;
    uartCfg.u8cDataBits       = DRVUART_DATABITS_8;
    uartCfg.u8cStopBits       = DRVUART_STOPBITS_1;
    uartCfg.u8cParity         = DRVUART_PARITY_NONE;
    uartCfg.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;
    uartCfg.u8TimeOut         = 0;
    DrvUART_Open(UART_CH, &uartCfg);

    /* --- LEDs --- */
    DrvGPIO_Open(LED_PORT, LED_GREEN_PIN,  E_IO_OUTPUT);
    DrvGPIO_Open(LED_PORT, LED_YELLOW_PIN, E_IO_OUTPUT);
    DrvGPIO_Open(LED_PORT, LED_RED_PIN,    E_IO_OUTPUT);
    DrvGPIO_ClrBit(LED_PORT, LED_GREEN_PIN);
    DrvGPIO_ClrBit(LED_PORT, LED_YELLOW_PIN);
    DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);

    /* --- Buzzer --- */
    DrvGPIO_Open(BUZZER_PORT, BUZZER_PIN, E_IO_OUTPUT);
    DrvGPIO_ClrBit(BUZZER_PORT, BUZZER_PIN);

    /* --- 7-Segment --- */
    DrvGPIO_Open(E_GPC, 4, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPC, 5, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPC, 6, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPC, 7, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 0, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 2, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 3, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 4, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 5, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 6, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 7, E_IO_OUTPUT);
    close_seven_segment();   /* blank all digits at boot */

    /* --- LCD + Keypad --- */
    init_LCD();
    clear_LCD();
    OpenKeyPad();
}

/*=============================================================================
    Keypad helper - debounces a raw ScanKey() value into a single edge-
    triggered press (fires once per press, not once per poll while held).
=============================================================================*/
static uint8_t DebounceKey(uint8_t rawKey)
{
    uint8_t pressed = KEY_NONE;

    if (rawKey != KEY_NONE && rawKey != g_lastKey)
        pressed = rawKey;

    g_lastKey = rawKey;
    return pressed;
}

static uint8_t GetNewKeyPress(void)
{
    return DebounceKey(ScanKey());
}

/*=============================================================================
    Game state handlers
=============================================================================*/
static void ResetMatch(void)
{
    g_myHP            = START_HP;
    g_enemyHP         = START_HP;
    g_prevEnemyHP     = START_HP;
    g_timeLeft        = MATCH_SECONDS;
    g_specialCooldown = 0;
    g_blockCooldown   = 0;
    g_blockActiveTicks= 0;
    g_lastKey         = KEY_NONE;

#if I_AM_BOY
    g_myX = 10;  g_enemyX = 85;
#else
    g_myX = 85;  g_enemyX = 10;
#endif
}

static void DoIntro(void)
{
    /* splash_screen is drawn once at each entry into STATE_INTRO (at boot,
       and on restart from the game-over screen) - see main() and
       DoGameOver() - so we just poll for a key press here, no repeated
       full-screen redraw. */
    if (GetNewKeyPress() != KEY_NONE)
    {
        ResetMatch();
        UART_SendPacket(CMD_START, 0);
        g_state = STATE_WAIT_OPPONENT;
        clear_LCD();
    }
}

static void DoWaitOpponent(void)
{
    uint8_t rxByte;
    uint32_t rxAvail;
    uint8_t got = 0;

    /* NOTE: this LCD only shows ~4 text lines (rows 0-3) at once, so the
       debug info below uses only those rows instead of 5/6 which sit
       off the bottom edge of the 64px-tall screen. */

    print_Line(0, "BATTLE ARENA");
    print_Line(1, "Waiting opp...");

    /* --- DEBUG: show raw bytes sitting in the UART RX FIFO -------------
       Remove this block once communication is confirmed working. -------*/
    rxAvail = _DRVUART_RECEIVEAVAILABLE(UART_CH);
    sprintf(g_lcdLine, "RX:%lu", (unsigned long)rxAvail);
    print_Line(2, g_lcdLine);
    /* ------------------------------------------------------------------*/

    /* keep re-announcing START in case the other board booted later */
    UART_SendPacket(CMD_START, 0);

    /* Handshake only needs to spot the CMD_START marker byte ANYWHERE in
       the incoming stream - scanning byte-by-byte instead of reading
       fixed 2-byte pairs means we can't get stuck out of alignment. */
    while (UART_TryReceiveByte(&rxByte))
    {
        /* --- DEBUG: show the last raw byte actually received --------- */
        sprintf(g_lcdLine, "Last:0x%02X", rxByte);
        print_Line(3, g_lcdLine);
        /* --------------------------------------------------------------*/

        if (rxByte == CMD_START)
        {
            got = 1;
            break;
        }
    }

    if (got)
    {
        g_state = STATE_COUNTDOWN;
        clear_LCD();
        return;
    }

    Delay_ms(200);   /* avoid flooding the UART/LCD while waiting */
}

static void DoCountdown(void)
{
    close_seven_segment();   /* blank leftover digit from previous match */

    draw_LCD((unsigned char *)starting_word);
    Buzzer_Beep(150, 0, 1);
    Delay_ms(900);

    draw_LCD((unsigned char *)digit_3);
    Buzzer_Beep(100, 0, 1);
    Delay_ms(700);

    draw_LCD((unsigned char *)digit_2);
    Buzzer_Beep(100, 0, 1);
    Delay_ms(700);

    draw_LCD((unsigned char *)digit_1);
    Buzzer_Beep(100, 0, 1);
    Delay_ms(700);

    draw_LCD((unsigned char *)start_word);
    Buzzer_Beep(300, 0, 1);
    Delay_ms(600);

    clear_LCD();
    RefreshStatusLine();
    DrawSprite(g_myX, MY_SPRITE);
    DrawSprite(g_enemyX, ENEMY_SPRITE);

    g_state = STATE_BATTLE;
}

static void ApplyIncomingDamage(uint8_t dmg)
{
    if (g_blockActiveTicks > 0)
        dmg = (uint8_t)(dmg / 2);          /* block halves damage */

    g_myHP -= dmg;
    if (g_myHP < 0) g_myHP = 0;

    Buzzer_Beep(60, 0, 1);                 /* "hit" beep */
    FlashDefeat(g_myX, MY_SPRITE, MY_DEFEAT);
    UART_SendPacket(CMD_HP_UPDATE, (uint8_t)g_myHP);
}

static void DoBattle_HandleAttackKey(uint8_t key)
{
    switch (key)
    {
        case KEY_PUNCH:
            if (InAttackRange())
                UART_SendPacket(CMD_PUNCH, DMG_PUNCH);
            break;

        case KEY_KICK:
            if (InAttackRange())
                UART_SendPacket(CMD_KICK, DMG_KICK);
            break;

        case KEY_SPECIAL:
            if (g_specialCooldown == 0 && InAttackRange())
            {
                UART_SendPacket(CMD_SPECIAL, DMG_SPECIAL);
                Buzzer_Beep(400, 0, 1);
                g_specialCooldown = SPECIAL_COOLDOWN_S;
            }
            break;

        case KEY_BLOCK:
            if (g_blockCooldown == 0)
            {
                g_blockActiveTicks = BLOCK_ACTIVE_S;
                g_blockCooldown    = BLOCK_COOLDOWN_S;
                UART_SendPacket(CMD_BLOCK, 0);
            }
            break;

        default:
            break;
    }
}

static void DoBattle_HandlePacket(uint8_t cmd, uint8_t data)
{
    switch (cmd)
    {
        case CMD_PUNCH:
        case CMD_KICK:
        case CMD_SPECIAL:
            ApplyIncomingDamage(data);
            break;

        case CMD_BLOCK:
            /* purely informational - opponent is shielding, nothing to do */
            break;

        case CMD_POS_UPDATE:
            UpdateEnemyPos(data);
            break;

        case CMD_HP_UPDATE:
            if (data < g_prevEnemyHP)
                FlashDefeat(g_enemyX, ENEMY_SPRITE, ENEMY_DEFEAT);
            g_prevEnemyHP = data;
            g_enemyHP     = data;
            break;

        case CMD_GAME_OVER:
            /* opponent ended the match (their HP hit 0, or their timer) */
            g_state = STATE_GAMEOVER;
            break;

        default:
            break;
    }
}

static void DoBattle(void)
{
    uint8_t rawKey, key, cmd, data;
    uint16_t chunk;

    /* --- one "second" of battle, sliced into short chunks so we keep
           polling keypad + UART instead of freezing for a whole second --- */
    for (chunk = 0; chunk < 10; chunk++)
    {
        rawKey = ScanKey();

        /* continuous movement while a direction key is held down */
        if (rawKey == KEY_LEFT)  MoveMe(-MOVE_STEP);
        if (rawKey == KEY_RIGHT) MoveMe(MOVE_STEP);

        /* edge-triggered for attacks so one press = one hit, not a
           machine-gun of hits for as long as the key is held */
        key = DebounceKey(rawKey);
        if (key == KEY_PUNCH || key == KEY_KICK ||
            key == KEY_SPECIAL || key == KEY_BLOCK)
            DoBattle_HandleAttackKey(key);

        if (UART_TryReceivePacket(&cmd, &data))
            DoBattle_HandlePacket(cmd, data);

        RefreshStatusLine();
        LED_UpdateForHP(g_myHP);

        if (g_myHP <= 0 || g_enemyHP <= 0)
            break;

        ShowTimer(g_timeLeft);   /* refresh every chunk (~10x/sec) so it stays lit */
        Delay_ms(100);           /* 10 x 100ms = ~1s per outer tick */
    }

    /* --- 1 Hz bookkeeping --- */
    if (g_timeLeft > 0) g_timeLeft--;
    if (g_specialCooldown > 0)  g_specialCooldown--;
    if (g_blockCooldown > 0)    g_blockCooldown--;
    if (g_blockActiveTicks > 0) g_blockActiveTicks--;

    if (g_myHP <= 0 || g_enemyHP <= 0 || g_timeLeft == 0)
    {
        UART_SendPacket(CMD_GAME_OVER, 0);
        g_state = STATE_GAMEOVER;
    }
}

static void DoGameOver(void)
{
    uint8_t iWon = 0, iLost = 0;

    close_seven_segment();   /* leave the display clean, no half-lit digit */

    if (g_myHP <= 0 && g_enemyHP > 0)          iLost = 1;
    else if (g_enemyHP <= 0 && g_myHP > 0)     iWon  = 1;
    else if (g_myHP > g_enemyHP)               iWon  = 1;
    else if (g_myHP < g_enemyHP)               iLost = 1;
    /* else: draw - neither flag set */

    if (iWon)
    {
        draw_LCD((unsigned char *)MY_WIN);
        Buzzer_Beep(120, 100, 3);
        Delay_ms(2000);
    }

    clear_LCD();
    print_Line(0, "GAME OVER");

    if (iWon)
        print_Line(1, "YOU WIN!");
    else if (iLost)
    {
        print_Line(1, "YOU LOSE");
        Buzzer_Beep(250, 150, 3);
    }
    else
    {
        print_Line(1, "DRAW");
        Buzzer_Beep(200, 200, 2);
    }

    print_Line(3, "Key = restart");

    DrvGPIO_SetBit(LED_PORT, LED_GREEN_PIN);
    DrvGPIO_SetBit(LED_PORT, LED_YELLOW_PIN);
    DrvGPIO_SetBit(LED_PORT, LED_RED_PIN);

    if (GetNewKeyPress() != KEY_NONE)
    {
        UART_SendPacket(CMD_RESTART, 0);
        DrvGPIO_ClrBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_YELLOW_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);
        g_state = STATE_INTRO;
        draw_LCD((unsigned char *)splash_screen);
    }
}

/*=============================================================================
    main
=============================================================================*/
int main(void)
{
    Hardware_Init();

    g_state = STATE_INTRO;
    draw_LCD((unsigned char *)splash_screen);

    while (1)
    {
        switch (g_state)
        {
            case STATE_INTRO:          DoIntro();          break;
            case STATE_WAIT_OPPONENT:  DoWaitOpponent();   break;
            case STATE_COUNTDOWN:      DoCountdown();      break;
            case STATE_BATTLE:         DoBattle();         break;
            case STATE_GAMEOVER:       DoGameOver();       break;
        }
    }
}
