#include <inttypes.h>
#include <stdint.h>
#include <sys/socket.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <setjmp.h>
#include <xtensa/xtensa_api.h>
#include <esp_task_wdt.h>
#include "exception_names.h"
#include "esp_camera.h"

typedef intptr_t cell_t;

extern "C" {
  void panic_print_str(const char *str);
  void panic_print_registers(const void *frame, int core);
}


__thread jmp_buf g_forth_fault;
__thread int g_forth_signal;
__thread uint32_t g_forth_setlevel;
void IRAM_ATTR forth_exception_handler(XtExcFrame *frame)
{
  g_forth_signal = frame->exccause;
  XTOS_RESTORE_INTLEVEL(g_forth_setlevel);

  panic_print_str("Exception: ");
  panic_print_str(exception_names[frame->exccause]);
  panic_print_str("\n");
  panic_print_registers(frame, ARDUINO_RUNNING_CORE);

  longjmp(g_forth_fault, 1);
}

void register_exception_handlers(void)
{
  uint32_t default_setlevel = XTOS_SET_INTLEVEL(XCHAL_EXCM_LEVEL);
  XTOS_RESTORE_INTLEVEL(default_setlevel);
  g_forth_setlevel = default_setlevel;

  for (int i = 0; i < 64; i++) {
    xt_set_exception_handler(i, forth_exception_handler);
  }
}


#define HEAP_SIZE (25 * 1024)
#define STACK_SIZE 128
#define FLAG_IMMEDIATE 1
#define WDT_TIMEOUT 10

#define DUP *++sp = tos
#define DROP tos = *sp--
#define COMMA(n) *g_sys.here++ = ((cell_t)n)
#define NEXT w = *ip++; goto **(void **)w
#define CELL_LEN(n) (((n) + sizeof(cell_t) - 1) / sizeof(cell_t))
#define FIND(name) find(name, sizeof(name) - 1)
#define LOWER(ch) ((ch) & 0x5F)

#define OPCODE_LIST \
  X("'DOCOL", TICKDOCOL, DUP; tos = (cell_t)&&OP_DOCOLON) \
  X("=",   EQUAL, tos = (*sp == tos) ? -1 : 0; --sp) \
  X("<",   LESS, tos = (*sp < tos) ? -1 : 0; --sp) \
  X("+",   ADD,  tos = *sp + tos; --sp) \
  X("-",   SUB,  tos = *sp - tos; --sp) \
  X("/",   DIV,  tos = *sp / tos; --sp) \
  X("U/",  UDIV, tos = (unsigned)*sp / tos; --sp) \
  X("MOD", MOD,  tos = *sp % tos; --sp) \
  X("UMOD",UMOD, tos = (unsigned)*sp % tos; --sp) \
  X("*",   MUL,  tos = *sp * tos; --sp) \
  X("AND", AND,  tos = *sp & tos; --sp) \
  X("OR",  OR,   tos = *sp | tos; --sp) \
  X("XOR", XOR,  tos = *sp ^ tos; --sp) \
  X("LSHIFT",  LSHIFT,  tos = *sp << tos; --sp) \
  X("RSHIFT",  RSHIFT,  tos = (unsigned)*sp >> tos; --sp) \
  X("OVER", OVER, DUP; tos = sp[-1]) \
  X("@",   FETCH,   tos = *(cell_t *)tos) \
  X("U@",  UFETCH,  tos = *(uint32_t *)tos) \
  X("W@",  WFETCH,  tos = *(int16_t *)tos) \
  X("UW@", UWFETCH, tos = *(uint16_t *)tos) \
  X("C@",  CFETCH,  tos = *(int8_t *)tos) \
  X("UC@", UCFETCH, tos = *(uint8_t *)tos) \
  X("!",   STORE, *(cell_t *)tos = *sp--; DROP) \
  X("W!", WSTORE, *(uint16_t *)tos = *sp--; DROP) \
  X("C!", CSTORE, *(uint8_t *)tos = *sp--; DROP) \
  X("SP@", SPFETCH, DUP; tos = (cell_t) sp) \
  X("SP!", SPSTORE, sp = (cell_t *) tos; DROP) \
  X("RP@", RPFETCH, DUP; tos = (cell_t) rp) \
  X("RP!", RPSTORE, rp = (cell_t *) tos; DROP) \
  X(",", COMMA_TOKEN, COMMA(tos); DROP) \
  X("'", TICK, DUP; DUP; tos = parse(' ', (cell_t *)sp); tos = find((const char *)*sp, tos); *sp = tos; DROP) \
  X("EXECUTE", EXECUTE, w = tos; DROP; goto **(void **)w) \
  X("BRANCH", BRANCH, ip = (cell_t *)*ip) \
  X("0BRANCH", ZBRANCH, if (!tos) ip = (cell_t *)*ip; else ++ip; DROP) \
  X("DOLIT", DOLIT, DUP; tos = *ip; ++ip) \
  X("FIND", FIND, tos = find((const char *)*sp, tos); --sp) \
  X("PARSE", PARSE, DUP; tos = parse(tos, (cell_t *)sp)) \
  X("(CREATE)", PAREN_CREATE, create((const char *)sp[-1], sp[0], (void *)tos); DROP; DROP; DROP) \
  X("IMMEDIATE", IMMEDIATE, g_sys.latest[-1] |= FLAG_IMMEDIATE) \
  X("'SYS", SYS, DUP; tos = (cell_t)&g_sys) \
  X("EVALUATE1", EVALUATE1, \
      DUP; sp = (cell_t *)evaluate1((cell_t *)sp); \
      w = *sp; --sp; DROP; \
      if (w) goto **(void **)w) \
  X("EXIT", EXIT, ip = (cell_t *)*rp; --rp) \
  X("SKEY", SKEY, while(!Serial.available()) {} DUP; tos = Serial.read()) \
  X("SKEY?", SKEY_Q, DUP; tos = Serial.available()) \
  X("STYPE", STYPE, {char buf[128];snprintf(buf, sizeof(buf), "%.*s", tos, (const uint8_t *)*sp);panic_print_str(buf);}/*Serial.write((const uint8_t *) *sp, tos); */--sp; DROP) /* to workaround deadlock when printing invalid strings */ \
  X("MS", MS, delay(tos); DROP) \
  X("BYE", BYE, ESP.restart()) \
  X("LEDCSETUP", LEDCSETUP, sp[-1] = ledcSetup(sp[-1], sp[0], tos); DROP; DROP) \
  X("LEDCATTACHPIN", LEDCATTACHPIN, ledcAttachPin(*sp, tos); DROP; DROP) \
  X("LEDCWRITE", LEDCWRITE, ledcWrite(*sp, tos); DROP; DROP) \
  X("WIFI.BEGIN", WIFIBEGIN, *sp = (cell_t)WiFi.begin(*(const char **)sp, (const char *)tos); DROP) \
  X("WIFI.STATUS", WIFISTATUS, DUP; tos = WiFi.status()) \
  X("WIFI.LOCALIP", WIFILOCALIP, DUP; tos = FromIP(WiFi.localIP())) \
  X("WIFI.RSSI", WIFIRSSI, DUP; tos = WiFi.RSSI()) \
  X("PINMODE", PINMODE, pinMode((uint8_t)*sp, (uint8_t)tos); DROP; DROP) \
  X("DIGITALWRITE", DIGITALWRITE, digitalWrite((uint8_t)*sp, (uint8_t)tos); DROP; DROP) \
  X("WRITE-FILE", WRITE_FILE, tos = write(sp[-1], (void *)sp[0], tos); sp -= 2) \
  X("READ-FILE", READ_FILE, tos = read(sp[-1], (void *)sp[0], tos); sp -= 2) \
  X("LISTEN", LISTEN, tos = listen(sp[0], tos); --sp) \
  X("SOCKACCEPT", SOCKACCEPT, tos = accept(sp[-1], (sockaddr *)sp[0], (socklen_t *)tos); sp -= 2) \
  X("SOCKET", SOCKET, tos = socket(sp[-1], sp[0], tos); sp -= 2) \
  X("BIND", BIND, tos = bind(sp[-1], (sockaddr *)sp[0], tos); sp -= 2) \
  X("NON-BLOCK", NON_BLOCK, tos = fcntl(tos, F_SETFL, O_NONBLOCK); tos = tos < 0 ? errno : 0) \
  X("CLOSE-FILE", CLOSE_FILE, tos = close(tos)) \
  X("MS-TICKS", MS_TICKS, DUP; tos = millis()) \
  X("ESP-CAMERA-INIT", ESP_CAMERA_INIT, tos = esp_camera_init((camera_config_t *)tos)) \
  X("ESP-CAMERA-DEINIT", ESP_CAMERA_DEINIT, DUP; tos = esp_camera_deinit()) \
  X("ESP-CAMERA-FB-GET", ESP_CAMERA_FB_GET, DUP; tos = (cell_t)esp_camera_fb_get()) \
  X("ESP-CAMERA-FB-RETURN", ESP_CAMERA_FB_RETURN, esp_camera_fb_return((camera_fb_t *)tos); DROP) \
  X("ESP-TASK-WDT-RESET", ESP_TASK_WDT_RESET, esp_task_wdt_reset()) \
  X("DD", DD, for(cell_t *start = (cell_t *)here + STACK_SIZE + STACK_SIZE; start < g_sys.here; start++) { printf("%08x: %08x\n", start, *start); } ) \

struct {
  const char *tib;
  cell_t ntib, tin, state, base;
  cell_t *here, *latest;
  cell_t NOTFOUND_XT, DOLIT_XT;
  cell_t **throw_handler;
} g_sys;

cell_t FromIP(IPAddress ip) {
  cell_t ret = 0;
  ret = (ret << 8) | ip[3];
  ret = (ret << 8) | ip[2];
  ret = (ret << 8) | ip[1];
  ret = (ret << 8) | ip[0];
  return ret;
}

cell_t parseint(const char *pos, cell_t n, cell_t *ret)
{
  *ret = 0;
  cell_t negate = 0;
  cell_t base = g_sys.base;
  if (!n) { return 0; }
  if (pos[0] == '-') { negate = 1; ++pos; --n; }
  if (pos[0] == '$') { base = 16; ++pos; --n; }
  if (pos[0] == '#') { base = 10; ++pos; --n; }
  if (pos[0] == '0' && n > 1 && pos[1] == 'x') { base = 16; pos += 2; n -= 2; }
  for (; n; --n) {
    uintptr_t d = pos[0] - '0';
    if (d > 9) {
      d = LOWER(d) - 7;
      if (d < 10) { return 0; }
    }
    if (d >= base) { return 0; }
    *ret = *ret * base + d;
    ++pos;
  }
  if (negate) { *ret = -*ret; }
  return -1;
}

cell_t same(const char *a, const char *b, cell_t len)
{
  for (;len && LOWER(*a) == LOWER(*b); --len, ++a, ++b);
  return len;
}

cell_t find(const char *name, cell_t len)
{
  cell_t *pos = g_sys.latest;
  cell_t clen = CELL_LEN(len);
  while (pos) {
    if (len == pos[-3] && same(name, (const char *) &pos[-3 - clen], len) == 0) {
      return (cell_t) pos;
    }
    pos = (cell_t *) pos[-2];  // Follow link
  }
  return 0;
}

/* dict format:
   [ name ]
   [ name ]
   [ name ]
   [ name len ]
   [ *prev cfa ]
   [ flags ]
   [ cfa ]  <- latest, "xt"
   [ words ]
   [ words ]
   [ words ]
*/

void create(const char *name, cell_t length, void *op)
{
  memcpy(g_sys.here, name, length);  // name
  g_sys.here += CELL_LEN(length);
  COMMA(length);  // length
  COMMA(g_sys.latest);  // link
  COMMA(0);  // flags
  g_sys.latest = g_sys.here;
  COMMA(op);  // code
}

int match(char sep, char ch)
{
  return sep == ch || (sep == ' ' && (ch == '\t' || ch == '\n' || ch == '\r'));
}

cell_t parse(cell_t sep, cell_t *ret)
{
  if (sep == ' ') {
    while (g_sys.tin < g_sys.ntib &&
           match(sep, g_sys.tib[g_sys.tin])) { ++g_sys.tin; }
  }
  cell_t start = g_sys.tin;
  while (g_sys.tin < g_sys.ntib &&
         !match(sep, g_sys.tib[g_sys.tin])) { ++g_sys.tin; }
  cell_t len = g_sys.tin - start;
  if (g_sys.tin < g_sys.ntib) { ++g_sys.tin; }
  *ret = (cell_t)(g_sys.tib + start);
  return len;
}

cell_t *evaluate1(cell_t *sp)
{
  cell_t call = 0;
  cell_t name;
  cell_t len = parse(' ', &name);
  cell_t xt = find((const char *)name, len);
  if (xt) {
    if (g_sys.state && !(((cell_t *)xt)[-1] & FLAG_IMMEDIATE)) {
      COMMA(xt);
    } else {
      call = xt;
    }
  } else {
    cell_t n;
    cell_t ok = parseint((const char *) name, len, &n);
    if (ok) {
      if (g_sys.state) {
        COMMA(g_sys.DOLIT_XT);
        COMMA(n);
      } else {
        *++sp = n;
      }
    } else {
      printf("not found: %.*s\n", len, name);
      *++sp = name;
      *++sp = len;
      *++sp = -1;
      call = g_sys.NOTFOUND_XT;
    }
  }
  *++sp = call;
  return sp;
}

void ueforth(void *here, const char *src, cell_t src_len)
{
  g_sys.here = (cell_t *)here;
  cell_t *sp = g_sys.here; g_sys.here += STACK_SIZE;
  cell_t *rp = g_sys.here; g_sys.here += STACK_SIZE;
  cell_t tos = 0, *ip, w;

#define X(name, op, code) create(name, sizeof(name) - 1, && OP_ ## op);
  OPCODE_LIST
#undef X

  g_sys.DOLIT_XT = FIND("DOLIT");
  /* g_sys.NOTFOUND_XT uninitialized */
  g_sys.base = 10;
  g_sys.state = 0;
  g_sys.tin = 0;
  g_sys.tib = src;
  g_sys.ntib = src_len;

  ip = g_sys.here;
  COMMA(FIND("EVALUATE1"));
  COMMA(FIND("BRANCH"));
  COMMA(ip);

  if (setjmp(g_forth_fault)) {
    delay(1000);
    rp = *g_sys.throw_handler;
    *g_sys.throw_handler = (cell_t *) *rp--;
    sp = (cell_t *) *rp--;
    ip = (cell_t *) *rp--;
    --sp;
    tos = -g_forth_signal;
  } else {
    register_exception_handlers();
  }

  /* start interpreting EVALUATE1/BRANCH loop defined above */
  NEXT;

#define X(name, op, code) OP_ ## op: { code; } NEXT;
  OPCODE_LIST
#undef X

  OP_DOCOLON: ++rp; *rp = (cell_t) ip; ip = (cell_t *) (w + sizeof(cell_t)); NEXT;
}

const char boot[] = R"(
32 PARSE header 'DOCOL (CREATE) ' DOLIT , 32 , ' PARSE , ' DOLIT , 'DOCOL , ' (CREATE) , ' EXIT ,
header : ' header , ' DOLIT , -1 , ' DOLIT , 'SYS 3 4 * + , ' ! , ' EXIT ,
header ; ' DOLIT , ' EXIT , ' , , ' DOLIT , 0 , ' DOLIT , 'SYS 3 4 * + , ' ! , ' EXIT , IMMEDIATE

: cell  4 ;
: cell+  cell + ;
: cell-  cell - ;
: cells  cell * ;
: cell/  cell / ;
: drop  sp@ cell- sp! ;
: (  41 parse drop drop ; immediate
: \  10 parse drop drop ; immediate
: dup ( a -- a a ) sp@ @ ;
: >r ( a -- ) ( R: -- a ) rp@ rp@ @ rp@ cell+ dup rp! ! ! ;
: r> ( -- a ) ( R: a -- ) rp@ cell- @ rp@ @ rp@  cell- dup rp! ! ;
: r@ ( -- a ) rp@ cell- @ ;
: swap ( a b -- b a ) sp@ cell- dup @ >r ! r> ;
: over ( a b -- a b a ) >r dup r> swap ;
: 2drop ( n n -- ) drop drop ;
: 2dup ( a b -- a b a b ) over over ;
: nip ( a b -- b ) swap drop ;
: rdrop ( R: n -- ) r> r> drop >r ;
: rot ( a b c -- c a b ) >r swap r> swap ;
: -rot ( a b c -- b c a ) swap >r swap r> ;
: pick  ( n -- n ) 1 + cells sp@ swap - @ ;
: rpick ( R: n -- n ) 1 + cells rp@ swap - @ ;
: u/mod ( n n -- n ) 2dup umod -rot u/ ;
: invert ( n -- ~n ) -1 xor ;
: negate ( n -- -n ) invert 1 + ;
: 0= ( n -- n ) 0 = ;
: 0< ( n -- n ) 0 < ;
: > ( a b -- a>b ) swap < ;
: >= ( n n -- n ) < invert ;
: <> ( a b -- a!=b ) = invert ;
: 1+ ( n -- n ) 1 + ;
: +! ( n a -- ) swap over @ + swap ! ;

\ System Variables
: 'tib ( -- a ) 'sys 0 cells + ;
: #tib ( -- a ) 'sys 1 cells + ;
: >in ( -- a ) 'sys 2 cells + ;
: state ( -- a ) 'sys 3 cells + ;
: base ( -- a ) 'sys 4 cells + ;
: 'here ( -- a ) 'sys 5 cells + ;
: 'latest ( -- a ) 'sys 6 cells + ;
: 'notfound ( -- a ) 'sys 7 cells + ;
: 'throw-handler ( -- a ) 'sys 9 cells + ;

\ Dictionary
: here ( -- a ) 'here @ ;
: latest ( -- a ) 'latest @ ;
: allot ( n -- ) 'here +! ;
: aligned ( a -- a ) cell+ 1 - cell negate and ;
: align  ( -- ) here aligned 'here ! ;
: c, ( ch -- ) here c! 1 allot ;

\ Compilation State
: [  0 state ! ; immediate
: ] -1 state ! ; immediate

\ Quoting Words
: bl  ( -- n ) 32 ;
: nl  ( -- n ) 10 ;
: aliteral [ ' dolit , ' dolit , ' , , ] , ;
: ['] ' aliteral ; immediate
: char bl parse drop c@ ;
: [char] char aliteral ; immediate
: literal aliteral ; immediate

\ Core Control Flow
: begin   here ; immediate
: again   ['] branch , , ; immediate
: until   ['] 0branch , , ; immediate
: ahead   ['] branch , here 0 , ; immediate
: then   here swap ! ; immediate
: if   ['] 0branch , here 0 , ; immediate
: else   ['] branch , here 0 , swap here swap ! ; immediate
: while   ['] 0branch , here 0 , swap ; immediate
: repeat   ['] branch , , here swap ! ; immediate
: aft   drop ['] branch , here 0 , here swap ; immediate

\ Compound words requiring conditionals
: min 2dup < if drop else nip then ;
: max 2dup < if nip else drop then ;
: abs ( n -- +n ) dup 0< if negate then ;
: ?dup  ( n -- 0 | n n ) dup if dup then ;

\ Dictionary Format
: >name ( xt -- a n ) 3 cells - dup @ swap over aligned - swap ;
: >link ( xt -- a ) 2 cells - @ ;
: >flags ( xt -- flags ) cell - ;
: >body ( xt -- a ) 2 cells + ;
: >:body ( xt -- a ) cell+ ;

\ Postpone - done here so we have ['] and IF
: immediate? ( xt -- f ) >flags @ 1 and 0= 0= ;
: postpone ' dup immediate? if , else aliteral ['] , , then ; immediate

\ create and does> (taken from planckforth)
: nop ;
: :noname 0 , latest , 0 , here dup 'latest ! 'docol , postpone ] ;
: create   header here 4 cells + aliteral postpone nop postpone exit ;
: (does>) ( n -- )  latest 3 cells + ! ;
: does>  0 aliteral here cell - postpone (does>) postpone ; :noname swap ! ; immediate

\ Constants and Variables
: constant ( n "name" -- ) create , does> @ ;
: variable ( "name" -- ) create 0 , ;

\ Counted Loops
variable nest-depth
variable leaving
: leaving,   here leaving @ , leaving ! ;
: leaving(   leaving @ 0 leaving !   2 nest-depth +! ;
: )leaving   leaving @ swap leaving !  -2 nest-depth +! begin dup while dup @ swap here swap ! repeat drop ;
: (do)   ( n n -- .. ) swap r> -rot >r >r >r ;
: do     ( lim s -- ) leaving( postpone (do) here ; immediate
: (?do)  ( n n -- n n f .. ) 2dup = if 2drop r> @ >r else swap r> cell+ -rot >r >r >r then ;
: ?do    ( lim s -- ) leaving( postpone (?do) leaving, here ; immediate
: unloop   r> rdrop rdrop >r ;
: (leave)   r> rdrop rdrop @ >r ;
: leave   postpone (leave) leaving, ; immediate
: (+loop)  ( n -- ) dup 0< swap r> r> rot + dup r@ < -rot >r >r xor 0= if r> cell+ rdrop rdrop >r else r> @ >r then ;
: +loop    ( n -- ) postpone (+loop) , )leaving ; immediate
: (loop)  r> r> 1+ dup r@ < -rot >r >r 0= if r> cell+ rdrop rdrop >r else r> @ >r then ;
: loop   postpone (loop) , )leaving ; immediate
: i 1 rpick ;
: j 3 rpick ;
: k 5 rpick ;

\ Stack Convenience
sp@ constant sp0
rp@ constant rp0
: depth ( -- n ) sp@ sp0 - cell/ ;

\ Exceptions
variable handler
handler 'throw-handler !
: catch ( xt -- n )  sp@ >r handler @ >r rp@ handler ! EXECUTE r> handler ! r> drop 0 ;
: throw ( n -- )     dup if handler @ rp! r> handler !  r> swap >r sp! drop r> else drop then ;

\ Values
: value ( n -- ) constant ;
: >value 5 cells + ;
: to ( n -- ) state @ if postpone ['] postpone >value postpone !
                      else ' >value ! then ; immediate

\ Deferred Words
: defer ( "name" -- ) create 0 , does> @ dup 0= throw EXECUTE ;
: is ( xt "name" -- ) postpone to ; immediate

defer key  ' SKEY is key
defer type  ' STYPE is type

: emit ( n -- ) >r rp@ 1 type rdrop ;
: space bl emit ;
: cr nl emit ;

\ Numeric Output
variable hld
: pad ( -- a ) here 80 + ;
: digit ( u -- c ) 9 over < 7 and + [char] 0 + ;
: extract ( n base -- n c ) u/mod swap digit ;
: <# ( -- ) pad hld ! ;
: hold ( c -- ) hld @ 1 - dup hld ! c! ;
: # ( u -- u ) base @ extract hold ;
: #s ( u -- 0 ) begin # dup while repeat ;
: sign ( n -- ) 0< if [char] - hold then ;
: #> ( w -- b u ) drop hld @ pad over - ;
: str ( n -- b u ) dup >r abs <# #s r> sign #> ;
: hex ( -- ) 16 base ! ;
: decimal ( -- ) 10 base ! ;
: u. ( u -- ) <# #s #> type space ;
: #n ( u n -- u )  0 ?do # loop ;
: u.0 ( u n -- )  <# 1 - #n #s #> type ;
: .. ( n -- ) str type ;
: . ( w -- ) base @ 10 xor if u. else .. space then ;
: u.8$  ( u -- ) base @ >r   hex 8 u.0    r> base ! ;
: ip# ( n -- n ) dup 255 and .. [char] . emit 8 rshift ;
: ip. ( n -- ) ip# ip# ip# 255 and . ;
: ip? ( -- ) wifi.localip ip. ;

\ Strings
: parse-quote ( -- a n ) [char] " parse ;
: $place ( a n -- ) 0 do dup c@ c, 1+ loop drop 0 c, align ;
: $@   r@ dup cell+ swap @ r> dup @ 1+ aligned + cell+ >r ;
: s"   parse-quote state @ if postpone $@ dup , $place
       else dup here swap >r >r $place r> r> then ; immediate
: ."   postpone s" state @ if postpone type else type then ; immediate
: z"   postpone s" state @ if postpone drop else drop then ; immediate

\ See
\ TODO: errors when decompiling:
\ - words containing ." and s"
\ - loops/0branch
: builtin? ( xt -- ) @ 'DOCOL <> ;
: see. ( xt -- ) dup >name nip if >name type space else ." noname@" u.8$ space then ;
: see-one ( xt -- xt+1 )  dup @ dup ['] DOLIT = if drop cell+ dup @ . else see. then cell+ ;
: exit= ( xt -- ) ['] exit = ;
: see-loop   >:body begin dup @ exit= invert while see-one repeat ;
: see-immediate?   immediate? if ."  immediate" then ;
: see-:   ['] : see.  dup see. space dup see-loop drop  ['] ; see. see-immediate? ;
: see   ' dup builtin? if dup see. ." is builtin" else see-: then cr ;
: words   latest begin dup see. >link dup 0= until drop cr ;

\ Dump
: printprim  ( a -- ) ['] dd begin dup @ 2 pick = if ." &&DO_" >name type drop exit then >link ?dup 0= until u.8$ ;
: printword  ( xt -- )  latest begin 2dup = if ." --> " see. drop exit then >link ?dup 0= until printprim ;
: dumpaddr  ( addr -- ) dup u.8$ space @ printword cr ;
: bounds over + swap ;
: dump  ( addr ncells -- ) cells bounds do i dumpaddr cell +loop ;
: rawdump ( a n -- )  cr 0 do i 16 mod 0= if cr then dup i + c@ 2 u.0 space loop drop cr ;

\ Input
200 constant input-limit
create input-buffer   input-limit allot
: accept ( a n -- n ) 0 swap begin 2dup < while
    key dup nl = if 2drop nip exit then
    >r rot r> over c! 1+ -rot swap 1+ swap repeat drop nip ;
: refill ( -- n ) input-buffer 'tib !
    'tib @ input-limit accept #tib ! 0 >in ! ;


\ REPL
: prompt   state @ if ."  compiled" else ."  ok" then cr ;
: underflow?   sp@ sp0 < if ." STACK UNDERFLOW " -4 throw then ;
: interpret   begin >in @ #tib @ < while EVALUATE1 underflow? repeat ;
: recurse  latest , ; immediate
: quit ( -- )
    sp0 sp! rp0 rp! postpone [
    begin
      prompt refill
      ['] interpret catch
      if ." ERROR" cr recurse then
    again ;

\ Misc
: .s ." < " depth . ." > " depth 0 ?do sp0 i 1 + cells + @ . loop cr ;
: forget  ' dup >name drop 'here ! >link 'latest ! ;
: ?exit ( flag -- ) if rdrop exit then ;

\ Tasks
variable task-list
: .tasks ( -- )  task-list @ begin dup 5 cells - see. @ dup task-list @ = until drop ;
: pause
  rp@ sp@ task-list @ cell+ !
  task-list @ @ task-list !
  task-list @ cell+ @ sp! rp! ;
: task ( xt dsz rsz "name" )
   create here >r 0 , 0 , ( link, sp )
   swap here cell+ r@ cell+ ! cells allot
   here r@ cell+ @ ! cells allot
   dup 0= if drop else
     here r@ cell+ @ @ ! ( set rp to point here )
     , postpone pause ['] branch , here 3 cells - ,
   then rdrop
   2 cells allot ; \ maybe bug above?
: start-task ( t -- )
   task-list @ if
     task-list @ @ over !
     task-list @ !
   else
     dup task-list !
     dup !
   then ;
: stop-task ( t -- ) task-list begin @ 2dup @ = until swap @ swap ! ;
: ms ( n -- ) MS-TICKS >r begin pause MS-TICKS r@ - over >= until rdrop drop ;

0 0 0 task main-task  main-task start-task

\ Motors
15 constant ena 32 constant in1 33 constant in2
13 constant enb 14 constant in3 12 constant in4

2 constant PWMCHAN_DRIVE
1 constant PWMCHAN_STEER
15974 constant PWMFREQ
2 constant PINMODE_OUTPUT
12 constant 12BIT
2000 value rampstrength
600 value drivestrength
80 value ramptime
500 value steptime
3000 value steerstrength

in1 PINMODE_OUTPUT pinmode in2 PINMODE_OUTPUT pinmode
in3 PINMODE_OUTPUT pinmode in4 PINMODE_OUTPUT pinmode

: hi ( n -- ) 1 digitalwrite ;
: lo ( n -- ) 0 digitalwrite ;

: steer  ( -- ) PWMCHAN_STEER steerstrength ledcwrite ;
: left   ( -- ) in3 hi  in4 lo  steer ;
: right  ( -- ) in3 lo  in4 hi  steer ;
: straight  ( -- ) PWMCHAN_STEER 0 ledcwrite ;

: stop  ( -- ) PWMCHAN_DRIVE 0 ledcwrite straight ;
: setdriveduty  ( n -- ) PWMCHAN_DRIVE swap ledcwrite ;
: step  ( -- )  rampstrength setdriveduty   ramptime ms   drivestrength setdriveduty  steptime ms   0 setdriveduty ;
: forw  ( -- ) in1 hi in2 lo ;
: back  ( -- ) in1 lo in2 hi ;
: motors-init
    PWMCHAN_DRIVE PWMFREQ 12bit ledcsetup drop
    PWMCHAN_STEER PWMFREQ 12bit ledcsetup drop
    ena PWMCHAN_DRIVE ledcattachpin
    enb PWMCHAN_STEER ledcattachpin
    stop ;

\ Sockets
1 constant SOCK_STREAM
2 constant AF_INET
16 constant sizeof(sockaddr_in)
: bs, ( n -- ) dup 8 rshift c, c, ;
: s, ( n -- ) dup c, 8 rshift c, ;
: l, ( n -- ) dup s, 16 rshift s, ;
: sockaddr   create sizeof(sockaddr_in) c, AF_INET c, 0 bs, 0 l, 0 l, 0 l, ;
: ->port! ( n a --  ) 2 + >r dup 8 rshift r@ c! r> 1+ c! ;

sockaddr serversock
: server-init ( port -- fd )
    serversock ->port!
    AF_INET SOCK_STREAM 0 SOCKET
    dup serversock sizeof(sockaddr_in) BIND throw
    dup 1 LISTEN throw
    dup NON-BLOCK throw ;

\ Telnet
-1 value telnet-sockfd
-1 value telnet-clientfd
sockaddr telnet-clientsock
variable client-len
variable telnet-c -1 telnet-c !

: telnet-client-connected ( -- n ) telnet-clientfd -1 <> ;
: telnet-client-reset ( -- ) telnet-clientfd CLOSE-FILE drop -1 to telnet-clientfd ;
: telnet-c-valid ( -- flag ) telnet-c @ -1 <> ;
: telnet-c-reset ( -- ) -1 telnet-c ! ;
: telnet-key ( -- n ) telnet-c uc@ telnet-c-reset ;
: telnet-handle-err ( flag -- ) if telnet-client-reset then ;
: telnet-tryreadc ( -- ) telnet-client-connected invert ?exit telnet-clientfd telnet-c 1 READ-FILE 0= telnet-handle-err ;
: telnet-try-type ( addr u -- ) telnet-client-connected 0= if 2drop exit then telnet-clientfd -rot WRITE-FILE 0< telnet-handle-err ;
: telnet-key? ( -- flag ) telnet-c-valid if -1 exit then telnet-c-reset telnet-tryreadc telnet-c-valid ;

: telnet-poll ( -- )
    telnet-client-connected ?exit
    telnet-sockfd telnet-clientsock client-len SOCKACCEPT
        to telnet-clientfd
    telnet-clientfd NON-BLOCK drop ;

\ Camera
-1 value camera-sockfd
-1 value camera-clientfd
sockaddr camera-clientsock

4 constant PIXFORMAT_JPEG
8 constant FRAMESIZE_VGA      ( 640x480 )
0 constant CAMERA_FB_IN_PSRAM
1 constant CAMERA_FB_IN_DRAM

\ settings for double-layer module, NOT WROVER-DEV
create camera-config
  -1 , ( pin_pwdn ) -1 , ( pin_reset ) 21 , ( pin_xclk )
  26 , ( pin_sscb_sda ) 27 , ( pin_sscb_scl )
  35 , 34 , 39 , 36 , 19 , 18 , 5 , 4 , ( pin_d7 - pin_d0 )
  25 , ( pin_vsync ) 23 , ( pin_href ) 22 , ( pin_pclk )
  20000000 , ( xclk_freq_hz )
  0 , ( ledc_timer ) 0 , ( ledc_channel )
  PIXFORMAT_JPEG , ( pixel_format )
  FRAMESIZE_VGA , ( frame_size )
  12 , ( jpeg_quality 0-63 low good )
  1 , ( fb_count )
  CAMERA_FB_IN_DRAM ,

: camera-init ( -- ) camera-config ESP-CAMERA-INIT drop ;

: send ( buf len -- ) camera-clientfd -rot WRITE-FILE drop ;
: client-emit ( c -- )  >r camera-clientfd rp@ 1 WRITE-FILE drop rdrop ;
: client-cr 13 client-emit 10 client-emit ;

512 constant bufsize
create httpbuf bufsize allot

: camera-send-image
    \ pretend to read the request..
    20 ms
    camera-clientfd httpbuf bufsize READ-FILE drop

    s" HTTP/1.1 200 OK" send client-cr
    s" Content-Type: image/jpeg" send client-cr
    s" Connection: close" send client-cr
    s" Access-Control-Allow-Origin: *" send client-cr
    client-cr

    ESP-CAMERA-FB-GET dup dup @ swap cell+ @ send
    ESP-CAMERA-FB-RETURN ;

: camera-poll
    camera-clientfd CLOSE-FILE drop
    -1 to camera-clientfd
    camera-sockfd camera-clientsock client-len SOCKACCEPT
    dup 0< if drop exit then
    to camera-clientfd
    camera-send-image ;


: wdt-reset begin ESP-TASK-WDT-RESET 5000 ms again ;

' camera-poll 20 20 task camera-task  camera-task start-task
' telnet-poll 20 20 task telnet-task  telnet-task start-task
' wdt-reset 20 20 task wdt-task  wdt-task start-task

:noname ( -- n ) \ task-friendly telnet-aware key
    begin
       pause
       SKEY? if SKEY exit then
       telnet-key? if telnet-key exit then
    again ; is key
:noname ( n n -- ) 2dup STYPE telnet-try-type ; is type


: wifi-init ." wifi init" cr z" 1A90FF" z" thepassword" wifi.begin drop ;


wifi-init
." motors-init: " motors-init cr
." camera-init: " camera-init cr
23 server-init to telnet-sockfd
1337 server-init to camera-sockfd

500 ms
ip?


\ Better Errors
: notfound ( a n n -- )  if cr ." ERROR: " type ."  NOT FOUND!" cr -1 throw then ;
' notfound 'notfound !
: ' bl parse 2dup find dup >r -rot r> 0= 'notfound @ EXECUTE 2drop ;

quit
)";


void setup()
{
  cell_t *heap = (cell_t *) malloc(HEAP_SIZE);
  Serial.begin(115200);
  esp_netif_init();
  WiFi.setSleep(false);

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  ueforth(heap, boot, sizeof(boot));
}

void loop()
{
}
