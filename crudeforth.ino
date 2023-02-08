#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <setjmp.h>
#include <xtensa/xtensa_api.h>
#include "exception_names.h"

typedef intptr_t cell_t;
typedef int64_t dcell_t;
typedef uint64_t udcell_t;

extern "C" {
  void panic_print_str(const char *str);
  void panic_print_registers(const void *frame, int core);
}


static __thread jmp_buf g_forth_fault;
static __thread int g_forth_signal;
static __thread uint32_t g_forth_setlevel;
static void IRAM_ATTR forth_exception_handler(XtExcFrame *frame)
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


#define HEAP_SIZE (100 * 1024)
#define STACK_SIZE 512
#define FLAG_IMMEDIATE 1

#define DUP *++sp = tos
#define DROP tos = *sp--
#define COMMA(n) *g_sys.here++ = ((cell_t)n)
#define IMMEDIATE() g_sys.latest[-1] |= FLAG_IMMEDIATE
#define UMSMOD ud = *(udcell_t *)&sp[-1]; \
               --sp; *sp = (cell_t)(ud % tos); \
               tos = (cell_t)(ud / tos)
#define SSMOD d = (dcell_t)*sp * (dcell_t)sp[-1]; \
              --sp; *sp = (cell_t) (((udcell_t)d) % tos); \
              tos = (cell_t)(d < 0 ? ~(~d / tos) : d / tos)

#define NEXT w = *ip++; goto **(void **)w
#define CELL_LEN(n) (((n) + sizeof(cell_t) - 1) / sizeof(cell_t))
#define FIND(name) find(name, sizeof(name) - 1)
#define LOWER(ch) ((ch) & 0x5F)

#define OPCODE_LIST \
  X("'DOCOL", TICKDOCOL, DUP; tos = (cell_t)&&OP_DOCOLON) \
  X("0=", ZEQUAL, tos = !tos ? -1 : 0) \
  X("0<", ZLESS, tos = tos < 0 ? -1 : 0) \
  X("+", PLUS, tos += *sp; --sp) \
  X("UM/MOD", UMSMOD, UMSMOD) \
  X("*/MOD", SSMOD, SSMOD) \
  X("*", MUL, tos *= *sp; --sp) \
  X("AND", AND, tos &= *sp; --sp) \
  X("OR", OR, tos |= *sp; --sp) \
  X("XOR", XOR, tos ^= *sp; --sp) \
  X("DUP", DUP, DUP) \
  X("SWAP", SWAP, w = tos; tos = *sp; *sp = w) \
  X("OVER", OVER, DUP; tos = sp[-1]) \
  X("DROP", DROP, DROP) \
  X("@", AT, tos = *(cell_t *)tos) \
  X("C@", CAT, tos = *(uint8_t *)tos) \
  X("!", STORE, *(cell_t *)tos = *sp; --sp; DROP) \
  X("C!", CSTORE, *(uint8_t *)tos = *sp; --sp; DROP) \
  X("SP@", SPAT, DUP; tos = (cell_t) sp) \
  X("SP!", SPSTORE, sp = (cell_t *) tos; DROP) \
  X("RP@", RPAT, DUP; tos = (cell_t) rp) \
  X("RP!", RPSTORE, rp = (cell_t *) tos; DROP) \
  X(">R", TOR, ++rp; *rp = tos; DROP) \
  X(",", COMMA_TOKEN, COMMA(tos); DROP) \
  X("'", TICK, DUP; DUP; tos = parse(' ', (cell_t *)sp); tos = find((const char *)*sp, tos); *sp = tos; DROP) \
  X("R>", FROMR, DUP; tos = *rp; --rp) \
  X("R@", RAT, DUP; tos = *rp) \
  X("EXECUTE", EXECUTE, w = tos; DROP; goto **(void **)w) \
  X("BRANCH", BRANCH, ip = (cell_t *)*ip) \
  X("0BRANCH", ZBRANCH, if (!tos) ip = (cell_t *)*ip; else ++ip; DROP) \
  X("DONEXT", DONEXT, *rp = (*rp - 1); if (*rp) ip = (cell_t *)*ip; else (--rp, ++ip)) \
  X("DOLIT", DOLIT, DUP; tos = *ip; ++ip) \
  X("ALITERAL", ALITERAL, COMMA(g_sys.DOLIT_XT); COMMA(tos); DROP) \
  X("CELL", CELL, DUP; tos = sizeof(cell_t)) \
  X("FIND", FIND, tos = find((const char *)*sp, tos); --sp) \
  X("PARSE", PARSE, DUP; tos = parse(tos, (cell_t *)sp)) \
  X("S>NUMBER?", CONVERT, tos = convert((const char *)*sp, tos, (cell_t *)sp); if (!tos) --sp) \
  X("HEADER", HEADER, DUP; DUP; tos = parse(' ', (cell_t *)sp); \
                      create((const char *)*sp, tos, &&OP_DOCOLON); \
                         --sp; DROP) \
  X("IMMEDIATE", IMMEDIATE, IMMEDIATE()) \
  X("'SYS", SYS, DUP; tos = (cell_t)&g_sys) \
  X("EVALUATE1", EVALUATE1, \
      DUP; sp = (cell_t *)evaluate1((cell_t *)sp); \
      w = *sp; --sp; DROP; \
      if (w) goto **(void **)w) \
  X("EXIT", EXIT, ip = (cell_t *)*rp; --rp) \
  X("key", KEY, while(!Serial.available()) {} DUP; tos = Serial.read()) \
  X("key?", KEY_Q, DUP; tos = Serial.available()) \
  X("type", TYPE, {char buf[128];snprintf(buf, sizeof(buf), "%.*s", tos, (const uint8_t *)*sp);panic_print_str(buf);}/*Serial.write((const uint8_t *) *sp, tos); */--sp; DROP) \
  X("ms", MS, delay(tos); DROP) \
  X("bye", BYE, ESP.restart()) \
  X("ledcSetup", LEDCSETUP, sp[-1] = ledcSetup(sp[-1], sp[0], tos); DROP; DROP) \
  X("ledcAttachPin", LEDCATTACHPIN, ledcAttachPin(*sp, tos); DROP; DROP) \
  X("ledcWrite", LEDCWRITE, ledcWrite(*sp, tos); DROP; DROP) \
  X("WiFi.begin", WIFIBEGIN, *sp = (cell_t)WiFi.begin(*(const char **)sp, (const char *)tos); DROP) \
  X("WiFi.status", WIFISTATUS, DUP; tos = WiFi.status()) \
  X("WiFi.localIP", WIFILOCALIP, DUP; tos = FromIP(WiFi.localIP())) \
  X("server.begin", SERVERBEGIN, server.begin(tos); DROP) \
  X("pinMode", PINMODE, pinMode((uint8_t)*sp, (uint8_t)tos); DROP; DROP) \
  X("digitalWrite", DIGITALWRITE, digitalWrite((uint8_t)*sp, (uint8_t)tos); DROP; DROP) \
  X("DD", DD, for(cell_t *start = (cell_t *)here + STACK_SIZE + STACK_SIZE; start < g_sys.here; start++) { printf("%08x: %08x\n", start, *start); } ) \

static struct {
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

WiFiServer server(80);

static cell_t convert(const char *pos, cell_t n, cell_t *ret) {
  *ret = 0;
  cell_t negate = 0;
  cell_t base = g_sys.base;
  if (!n) { return 0; }
  if (pos[0] == '-') { negate = -1; ++pos; --n; }
  if (pos[0] == '$') { base = 16; ++pos; --n; }
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

static cell_t same(const char *a, const char *b, cell_t len) {
  for (;len && LOWER(*a) == LOWER(*b); --len, ++a, ++b);
  return len;
}

static cell_t find(const char *name, cell_t len) {
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

static void create(const char *name, cell_t length, void *op) {
  memcpy(g_sys.here, name, length);  // name
  g_sys.here += CELL_LEN(length);
  COMMA(length);  // length
  COMMA(g_sys.latest);  // link
  COMMA(0);  // flags
  g_sys.latest = g_sys.here;
  COMMA(op);  // code
}

static char spacefilter(char ch) {
  return ch == '\t' || ch == '\n' || ch == '\r' ? ' ' : ch;
}

static cell_t parse(cell_t sep, cell_t *ret) {
  while (g_sys.tin < g_sys.ntib &&
         spacefilter(g_sys.tib[g_sys.tin]) == sep) { ++g_sys.tin; }
  *ret = (cell_t) (g_sys.tib + g_sys.tin);
  while (g_sys.tin < g_sys.ntib &&
         spacefilter(g_sys.tib[g_sys.tin]) != sep) { ++g_sys.tin; }
  cell_t len = g_sys.tin - (*ret - (cell_t) g_sys.tib);
  if (g_sys.tin < g_sys.ntib) { ++g_sys.tin; }
  return len;
}

static cell_t *evaluate1(cell_t *sp) {
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
    cell_t ok = convert((const char *) name, len, &n);
    if (ok) {
      if (g_sys.state) {
        COMMA(g_sys.DOLIT_XT);
        COMMA(n);
      } else {
        *++sp = n;
      }
    } else {
      *++sp = name;
      *++sp = len;
      *++sp = -1;
      call = g_sys.NOTFOUND_XT;
    }
  }
  *++sp = call;
  return sp;
}

static void ueforth(void *here, const char *src, cell_t src_len) {
  g_sys.here = (cell_t *)here;
  register cell_t *sp = g_sys.here; g_sys.here += STACK_SIZE;
  register cell_t *rp = g_sys.here; g_sys.here += STACK_SIZE;
  register cell_t tos = 0, *ip, w;
  dcell_t d;
  udcell_t ud;

#define X(name, op, code) create(name, sizeof(name) - 1, && OP_ ## op);
  OPCODE_LIST
#undef X

  g_sys.DOLIT_XT = FIND("DOLIT");
  g_sys.NOTFOUND_XT = FIND("DROP");
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
HEADER : ' HEADER , -1 ALITERAL 'sys 3 cell * + ALITERAL ' ! , ' exit ,
HEADER ; ' exit ALITERAL ' , , 0 ALITERAL 'sys 3 cell * + ALITERAL ' ! , ' exit , IMMEDIATE

: bl 32 ;
: nl 10 ;
: (   41 parse drop drop ; immediate
( : \  nl parse drop drop ; immediate \ broken )
: 2drop ( n n -- ) drop drop ;
: 2dup ( a b -- a b a b ) over over ;
: nip ( a b -- b ) swap drop ;
: rdrop ( r: n n -- ) r> r> drop >r ;
: */ ( n n n -- n ) */mod nip ;
: /mod ( n n -- n n ) 1 swap */mod ;
: / ( n n -- n ) /mod nip ;
: mod ( n n -- n ) /mod drop ;
: invert ( n -- ~n ) -1 xor ;
: negate ( n -- -n ) invert 1 + ;
: - ( n n -- n ) negate + ;
: rot ( a b c -- c a b ) >r swap r> swap ;
: -rot ( a b c -- b c a ) swap >r swap r> ;
: < ( a b -- a<b ) - 0< ;
: > ( a b -- a>b ) swap - 0< ;
: = ( a b -- a!=b ) - 0= ;
: <> ( a b -- a!=b ) = 0= ;
: 1+ 1 + ;
: 1- 1 - ;
: +! ( n a -- ) swap over @ + swap ! ;


( \ Cells  )
: cell+ ( n -- n ) cell + ;
: cells ( n -- n ) cell * ;
: cell/ ( n -- n ) cell / ;

( \ System Variables )
: 'tib ( -- a ) 'sys 0 cells + ;
: #tib ( -- a ) 'sys 1 cells + ;
: >in ( -- a ) 'sys 2 cells + ;
: state ( -- a ) 'sys 3 cells + ;
: base ( -- a ) 'sys 4 cells + ;
: 'here ( -- a ) 'sys 5 cells + ;
: 'latest ( -- a ) 'sys 6 cells + ;
: 'notfound ( -- a ) 'sys 7 cells + ;
: 'throw-handler ( -- a ) 'sys 9 cells + ;

( \ Dictionary )
: here ( -- a ) 'here @ ;
: latest  'latest @ ;
: allot ( n -- ) 'here +! ;
: aligned ( a -- a ) cell+ 1- cell negate and ;
: align   here aligned 'here ! ;
: c, ( ch -- ) here c! 1 allot ;

( \ Compilation State )
: [  0 state ! ; immediate
: ] -1 state ! ; immediate

( \ Quoting Words )
( \ : ' bl parse 2dup find dup >r -rot r> 0= 'notfound @ execute 2drop ; )
: ['] ' aliteral ; immediate
: char bl parse drop c@ ;
: [char] char aliteral ; immediate
: literal aliteral ; immediate

( \ Core Control Flow  )
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

( \ Compound words requiring conditionals  )
: min 2dup < if drop else nip then ;
: max 2dup < if nip else drop then ;
: abs ( n -- +n ) dup 0< if negate then ;

( \ Dictionary Format  )
: >name ( xt -- a n ) 3 cells - dup @ swap over aligned - swap ;
: >link ( xt -- a ) 2 cells - @ ;
: >flags ( xt -- flags ) cell - ;
: >body ( xt -- a ) 2 cells + ;
: >:body ( xt -- a ) cell+ ;

( \ Postpone - done here so we have ['] and IF  )
: immediate? ( xt -- f ) >flags @ 1 and 0= 0= ;
: postpone ' dup immediate? if , else aliteral ['] , , then ; immediate

( \ Counted Loops  )
: for   postpone >r postpone begin ; immediate
: next   postpone donext , ; immediate
: dostart ( n n -- .. ) swap r> -rot >r >r >r ;
: docheck ( n -- f .. ) r> r> rot + dup r@ < -rot >r >r ;
: do   postpone dostart here 0 ; immediate
: ?do   postpone dostart 0 aliteral postpone ahead here swap ; immediate
: i   postpone r@ ; immediate
: j   rp@ 3 cells - @ ;
: unloop   postpone rdrop postpone rdrop ; immediate
: +loop   dup if postpone then else drop then
           postpone docheck postpone 0= postpone until
           postpone unloop ; immediate
: loop   1 aliteral postpone +loop ; immediate

( \ create and does> (taken from planckforth) )
: nop ;
: :noname 0 , latest , 0 , here dup 'latest ! 'docol , postpone ] ;
: create   header here 4 cells + aliteral postpone nop postpone exit ;
: (does>) ( n -- )  latest 3 cells + ! ;
: does>  0 aliteral here cell - postpone (does>) postpone ; :noname swap ! ; immediate

( \ Constants and Variables  )
: constant ( n "name" -- ) create , does> @ ;
: variable ( "name" -- ) create 0 , ;

( \ Stack Convience  )
sp@ constant sp0
rp@ constant rp0
: depth ( -- n ) sp@ sp0 - cell/ ;

( \ Exceptions  )
variable handler
handler 'throw-handler !
: catch ( xt -- n )  sp@ >r handler @ >r rp@ handler ! execute r> handler ! r> drop 0 ;
: throw ( n -- )     dup if handler @ rp! r> handler !  r> swap >r sp! drop r> else drop then ;
' throw 'notfound !

( \ Values  )
: value ( n -- ) create , does> @ ;
: to ( n -- ) state @ if postpone ['] postpone >body postpone !
                      else ' >body ! then ; immediate

( \ Deferred Words  )
: defer ( "name" -- ) create 0 , does> @ dup 0= throw execute ;
: is ( xt "name" -- ) postpone to ; immediate

: emit ( n -- ) >r rp@ 1 type rdrop ;
: space bl emit ;
: cr nl emit ;

( \ Numeric Output  )
variable hld
: pad ( -- a ) here 80 + ;
: digit ( u -- c ) 9 over < 7 and + 48 + ;
: extract ( n base -- n c ) 0 swap um/mod swap digit ;
: <# ( -- ) pad hld ! ;
: hold ( c -- ) hld @ 1 - dup hld ! c! ;
: # ( u -- u ) base @ extract hold ;
: #s ( u -- 0 ) begin # dup while repeat ;
: sign ( n -- ) 0< if 45 hold then ;
: #> ( w -- b u ) drop hld @ pad over - ;
: str ( n -- b u ) dup >r abs <# #s r> sign #> ;
: hex ( -- ) 16 base ! ;
: decimal ( -- ) 10 base ! ;
: u. ( u -- ) <# #s #> type space ;
: #n ( u n -- u )  0 ?do # loop ;
: u.0 ( u n -- )  <# 1- #n #s #> type ;
: . ( w -- ) base @ 10 xor if u. else str type space then ;
: ? ( a -- ) @ . ;

( \ Strings  )
: parse-quote ( -- a n ) [char] " parse ;
: $place ( a n -- ) 0 do dup c@ c, 1+ loop drop 0 c, align ;
: $@   r@ dup cell+ swap @ r> dup @ 1+ aligned + cell+ >r ;
: s"   parse-quote state @ if postpone $@ dup , $place
       else dup here swap >r >r $place r> r> then ; immediate
: ."   postpone s" state @ if postpone type else type then ; immediate
: z"   postpone s" state @ if postpone drop else drop then ; immediate

( \ Better Errors  )
: notfound ( a n n -- )  if cr ." ERROR: " type ."  NOT FOUND!" cr -1 throw then ;
' notfound 'notfound !

( \ Examine Dictionary  )
( \ TODO: crashes when decompiling: )
( \ - words containing ." and s" )
( \ - loops/0branch )
: builtin? ( xt -- ) @ 'DOCOL <> ;
: see. ( xt -- ) >name type space ;
: see-one ( xt -- xt+1 )  dup @ dup ['] DOLIT = if drop cell+ dup @ . else see. then cell+ ;
: exit= ( xt -- ) ['] exit = ;
: see-loop   >:body begin dup @ exit= invert while see-one repeat ;
: see-immediate?   immediate? if ."  immediate" then ;
: see-:   ['] : see.  dup see. space dup see-loop drop  ['] ; see.  see-immediate? ;
: see   ' dup builtin? if dup see. ." is builtin" else see-: then cr ;
: words   latest begin dup see. >link dup 0= until drop cr ;

( \ Examine Memory  )
: dump ( a n -- )  cr 0 do i 16 mod 0= if cr then dup i + c@ 2 u.0 space loop drop cr ;

( \ Input  )
: accept ( a n -- n ) 0 swap begin 2dup < while
   key dup nl = if 2drop nip exit then
   >r rot r> over c! 1+ -rot swap 1+ swap repeat drop nip ;
200 constant input-limit
: tib ( -- a ) 'tib @ ;
create input-buffer   input-limit allot
: tib-setup   input-buffer 'tib ! ;
: refill   tib-setup tib input-limit accept #tib ! 0 >in ! -1 ;

( \ REPL  )
: prompt   state @ if ."  compiled" else ."  ok" then cr ;
: evaluate-buffer   begin >in @ #tib @ < while evaluate1 repeat ;
: evaluate ( a n -- ) 'tib @ >r #tib @ >r >in @ >r
                      #tib ! 'tib ! 0 >in ! evaluate-buffer
                      r> >in ! r> #tib ! r> 'tib ! ;
: query   begin ['] evaluate-buffer catch
          if 0 state ! sp0 sp! rp0 rp! ." ERROR" cr then
          prompt refill drop again ;
: ok   ." uEForth" cr prompt refill drop query ;
: .s ." < " depth . ." > " depth 0 = if exit then depth 0 do sp0 i 1 + cells + @ . loop cr ;
: forget  ' dup >name drop 'here ! >link 'latest ! ;


: ?dup  ( n -- 0 | n n )  dup if dup then ;
: pick  ( n -- n ) 1+ cells sp@ swap - @ ;
: printprim  ( a -- ) ['] dd begin dup @ 2 pick = if ." && DO_" >name type drop exit then >link ?dup 0= until 8 u.0 ;
: printword  ( xt -- )  latest begin 2dup = if ." --> " >name type drop exit then >link ?dup 0= until printprim ( 8 u.0 ) ;
: showaddr  ( addr -- ) dup 8 u.0 space @ printword cr ;
: bounds over + swap ;
: dump  ( addr n-cells -- ) cells bounds do i showaddr cell +loop ;



200 ms
ok
)";


void setup() {
  cell_t *heap = (cell_t *) malloc(HEAP_SIZE);
  Serial.begin(115200);
  ueforth(heap, boot, sizeof(boot));
}

void loop() {
}
