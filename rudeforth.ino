#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef intptr_t cell_t;
typedef int64_t dcell_t;
typedef uint64_t udcell_t;


#define PUSH DUP; tos =
#define DUP *++sp = tos
#define DROP tos = *sp--
#define COMMA(n) *g_sys.heap++ = (n)
#define IMMEDIATE() g_sys.last[-1] |= 1
#define DOES(ip) *g_sys.last = (cell_t) && OP_DODOES; g_sys.last[1] = (cell_t) ip
#define UMSMOD ud = *(udcell_t *) &sp[-1]; \
               --sp; *sp = (cell_t) (ud % tos); \
               tos = (cell_t) (ud / tos)
#define SSMOD d = (dcell_t) *sp * (dcell_t) sp[-1]; \
              --sp; *sp = (cell_t) (((udcell_t) d) % tos); \
              tos = (cell_t) (d < 0 ? ~(~d / tos) : d / tos)

#define OPCODE_LIST \
  X("0=", ZEQUAL, tos = !tos ? -1 : 0) \
  X("0<", ZLESS, tos = (tos|0) < 0 ? -1 : 0) \
  X("+", PLUS, tos = (tos + *sp) | 0; --sp) \
  X("UM/MOD", UMSMOD, UMSMOD) \
  X("*/MOD", SSMOD, SSMOD) \
  X("AND", AND, tos = tos & *sp; --sp) \
  X("OR", OR, tos = tos | *sp; --sp) \
  X("XOR", XOR, tos = tos ^ *sp; --sp) \
  X("DUP", DUP, DUP) \
  X("SWAP", SWAP, w = tos; tos = (*sp)|0; *sp = w) \
  X("OVER", OVER, DUP; tos = sp[-1] | 0) \
  X("DROP", DROP, DROP) \
  X("@", AT, tos = (*(cell_t *) tos)|0) \
  X("L@", LAT, tos = (*(int32_t *) tos)|0) \
  X("C@", CAT, tos = (*(uint8_t *) tos)|0) \
  X("!", STORE, *(cell_t *) tos = (*sp)|0; --sp; DROP) \
  X("L!", LSTORE, *(int32_t *) tos = (*sp)|0; --sp; DROP) \
  X("C!", CSTORE, *(uint8_t *) tos = (*sp)|0; --sp; DROP) \
  X("FILL", FILL, memset((void *) (sp[-1] | 0), tos | 0, (*sp | 0)); sp -= 2; DROP) \
  X("MOVE", MOVE, memmove((void *) (sp[-1] | 0), (void *) (*sp | 0), tos | 0); sp -= 2; DROP) \
  X("SP@", SPAT, DUP; tos = (cell_t) sp) \
  X("SP!", SPSTORE, sp = (cell_t *) tos; DROP) \
  X("RP@", RPAT, DUP; tos = (cell_t) rp) \
  X("RP!", RPSTORE, rp = (cell_t *) tos; DROP) \
  X(">R", TOR, ++rp; *rp = tos; DROP) \
  X("R>", FROMR, DUP; tos = (*rp)|0; --rp) \
  X("R@", RAT, DUP; tos = (*rp)|0) \
  X("EXECUTE", EXECUTE, w = tos; DROP; goto **(void **) w) \
  X("BRANCH", BRANCH, ip = (cell_t *) (*ip | 0)) \
  X("0BRANCH", ZBRANCH, if (!tos) ip = (cell_t *) (*ip | 0); else ++ip; DROP) \
  X("DONEXT", DONEXT, *rp = ((*rp|0) - 1) | 0; \
                      if ((*rp|0)) ip = (cell_t *) (*ip | 0); else (--rp, ++ip)) \
  X("DOLIT", DOLIT, DUP; tos = (*ip | 0); ++ip) \
  X("ALITERAL", ALITERAL, COMMA(g_sys.DOLIT_XT | 0); COMMA(tos | 0); DROP) \
  X("CELL", CELL, DUP; tos = sizeof(cell_t)) \
  X("FIND", FIND, tos = find((const char *) (*sp | 0), tos|0)|0; --sp) \
  X("PARSE", PARSE, DUP; tos = parse(tos|0, (cell_t *) ((cell_t) sp | 0))|0) \
  X("S>NUMBER?", CONVERT, \
      tos = convert((const char *) (*sp | 0), tos|0, (cell_t *) ((cell_t) sp | 0))|0; \
      if (!tos) --sp) \
  X("CREATE", CREATE, DUP; DUP; tos = parse(32, (cell_t *) ((cell_t) sp | 0))|0; \
                      create((const char *) (*sp | 0), tos|0, 0, && OP_DOCREATE); \
                         COMMA(0); --sp; DROP) \
  X("DOES>", DOES, DOES((cell_t *) ((cell_t) ip|0)); ip = (cell_t *) (*rp | 0); --rp) \
  X("IMMEDIATE", IMMEDIATE, IMMEDIATE()) \
  X("'SYS", SYS, DUP; tos = (cell_t) &g_sys) \
  X(":", COLON, DUP; DUP; tos = parse(32, (cell_t *) ((cell_t) sp | 0))|0; \
                create((const char *) (*sp | 0), tos|0, 0, && OP_DOCOLON); \
                   g_sys.state = -1; --sp; DROP) \
  X("EVALUATE1", EVALUATE1, \
      DUP; sp = (cell_t *) ((cell_t) evaluate1((cell_t *) ((cell_t) sp | 0))|0); \
      w = (*sp | 0); --sp; DROP; \
      if (w) goto **(void **) w) \
  X("EXIT", EXIT, ip = (cell_t *) (*rp | 0); --rp) \
  X(";", SEMICOLON, COMMA(g_sys.DOEXIT_XT | 0); g_sys.state = 0) \



#include "SPIFFS.h"
#include <WiFi.h>
#include <WebServer.h>

#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

# define HEAP_SIZE (100 * 1024)
# define STACK_SIZE 512

#define PLATFORM_OPCODE_LIST \
  /* Serial */ \
  X("Serial.begin", SERIAL_BEGIN, Serial.begin(tos); DROP) \
  X("Serial.end", SERIAL_END, Serial.end()) \
  X("Serial.available", SERIAL_AVAILABLE, DUP; tos = Serial.available()) \
  X("Serial.readBytes", SERIAL_READ_BYTES, tos = Serial.readBytes((uint8_t *) *sp, tos); --sp) \
  X("Serial.write", SERIAL_WRITE, tos = Serial.write((const uint8_t *) *sp, tos); --sp) \
  /* Pins and PWM */ \
  X("pinMode", PIN_MODE, pinMode(*sp, tos); --sp; DROP) \
  X("digitalWrite", DIGITAL_WRITE, digitalWrite(*sp, tos); --sp; DROP) \
  X("analogRead", ANALOG_READ, tos = (cell_t) analogRead(tos)) \
  X("ledcSetup", LEDC_SETUP, \
      tos = (cell_t) (1000000 * ledcSetup(sp[-1], *sp / 1000.0, tos)); sp -= 2) \
  X("ledcAttachPin", ATTACH_PIN, ledcAttachPin(*sp, tos); --sp; DROP) \
  X("ledcDetachPin", DETACH_PIN, ledcDetachPin(tos); DROP) \
  X("ledcRead", LEDC_READ, tos = (cell_t) ledcRead(tos)) \
  X("ledcReadFreq", LEDC_READ_FREQ, tos = (cell_t) (1000000 * ledcReadFreq(tos))) \
  X("ledcWrite", LEDC_WRITE, ledcWrite(*sp, tos); --sp; DROP) \
  X("ledcWriteTone", LEDC_WRITE_TONE, \
      tos = (cell_t) (1000000 * ledcWriteTone(*sp, tos / 1000.0)); --sp) \
  X("ledcWriteNote", LEDC_WRITE_NOTE, \
      tos = (cell_t) (1000000 * ledcWriteNote(sp[-1], (note_t) *sp, tos)); sp -=2) \
  X("MS", MS, delay(tos); DROP) \
  X("TERMINATE", TERMINATE, exit(tos)) \
  /* File words */ \
  X("R/O", R_O, *++sp = O_RDONLY) \
  X("R/W", R_W, *++sp = O_RDWR) \
  X("W/O", W_O, *++sp = O_WRONLY) \
  X("BIN", BIN, ) \
  X("CLOSE-FILE", CLOSE_FILE, tos = close(tos); tos = tos ? errno : 0) \
  X("OPEN-FILE", OPEN_FILE, cell_t mode = tos; DROP; cell_t len = tos; DROP; \
    memcpy(filename, (void *) tos, len); filename[len] = 0; \
    tos = open(filename, mode, 0777); PUSH tos < 0 ? errno : 0) \
  X("CREATE-FILE", CREATE_FILE, cell_t mode = tos; DROP; cell_t len = tos; DROP; \
    memcpy(filename, (void *) tos, len); filename[len] = 0; \
    tos = open(filename, mode | O_CREAT | O_TRUNC); PUSH tos < 0 ? errno : 0) \
  X("DELETE-FILE", DELETE_FILE, cell_t len = tos; DROP; \
    memcpy(filename, (void *) tos, len); filename[len] = 0; \
    tos = unlink(filename); tos = tos ? errno : 0) \
  X("WRITE-FILE", WRITE_FILE, cell_t fd = tos; DROP; cell_t len = tos; DROP; \
    tos = write(fd, (void *) tos, len); tos = tos != len ? errno : 0) \
  X("READ-FILE", READ_FILE, cell_t fd = tos; DROP; cell_t len = tos; DROP; \
    tos = read(fd, (void *) tos, len); PUSH tos != len ? errno : 0) \
  X("FILE-POSITION", FILE_POSITION, \
    tos = (cell_t) lseek(tos, 0, SEEK_CUR); PUSH tos < 0 ? errno : 0) \
  X("REPOSITION-FILE", REPOSITION_FILE, cell_t fd = tos; DROP; \
    tos = (cell_t) lseek(fd, tos, SEEK_SET); tos = tos < 0 ? errno : 0) \
  X("FILE-SIZE", FILE_SIZE, struct stat st; w = fstat(tos, &st); \
    tos = (cell_t) st.st_size; PUSH w < 0 ? errno : 0) \
  /* WiFi */ \
  X("WiFi.config", WIFI_CONFIG, \
      WiFi.config(ToIP(sp[-1]), ToIP(*sp), ToIP(tos)); sp -= 2; DROP) \
  X("WiFi.begin", WIFI_BEGIN, \
      WiFi.begin((const char *) *sp, (const char *) tos); --sp; DROP) \
  X("WiFi.disconnect", WIFI_DISCONNECT, WiFi.disconnect()) \
  X("WiFi.status", WIFI_STATUS, DUP; tos = WiFi.status()) \
  X("WiFi.macAddress", WIFI_MAC_ADDRESS, WiFi.macAddress((uint8_t *) tos); DROP) \
  X("WiFi.localIP", WIFI_LOCAL_IPS, DUP; tos = FromIP(WiFi.localIP())) \
  /* SPIFFS */ \
  X("SPIFFS.begin", SPIFFS_BEGIN, tos = SPIFFS.begin(tos)) \
  X("SPIFFS.end", SPIFFS_END, SPIFFS.end()) \
  X("SPIFFS.format", SPIFFS_FORMAT, DUP; tos = SPIFFS.format()) \
  X("SPIFFS.totalBytes", SPIFFS_TOTAL_BYTES, DUP; tos = SPIFFS.totalBytes()) \
  X("SPIFFS.usedBytes", SPIFFS_USED_BYTES, DUP; tos = SPIFFS.usedBytes()) \
  /* WebServer */ \
  X("WebServer.new", WEBSERVER_NEW, DUP; tos = (cell_t) new WebServer(tos)) \
  X("WebServer.delete", WEBSERVER_DELETE, delete (WebServer *) tos; DROP) \
  X("WebServer.begin", WEBSERVER_BEGIN, \
      WebServer *ws = (WebServer *) tos; DROP; ws->begin(tos); DROP) \
  X("WebServer.stop", WEBSERVER_STOP, \
      WebServer *ws = (WebServer *) tos; DROP; ws->stop()) \

// TODO: Why doesn't ftruncate exist?
//  X("RESIZE-FILE", RESIZE_FILE, cell_t fd = tos; DROP; \
//    tos = ftruncate(fd, tos); tos = tos < 0 ? errno : 0) \

static char filename[PATH_MAX];

#define PRINT_ERRORS 0

#define NEXT w = *ip++; goto **(void **) w
#define CELL_LEN(n) (((n) + sizeof(cell_t) - 1) / sizeof(cell_t))
#define FIND(name) find(name, sizeof(name) - 1)
#define LOWER(ch) ((ch) & 0x5F)

#if PRINT_ERRORS
#include <unistd.h>
#endif

static struct {
  const char *tib;
  cell_t ntib, tin, state, base;
  cell_t *heap, *last, notfound;
  int argc;
  char **argv;
  cell_t DOLIT_XT, DOEXIT_XT;
} g_sys;

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
  cell_t *pos = g_sys.last;
  cell_t clen = CELL_LEN(len);
  while (pos) {
    if (len == pos[-3] &&
        same(name, (const char *) &pos[-3 - clen], len) == 0) {
      return (cell_t) pos;
    }
    pos = (cell_t *) pos[-2];  // Follow link
  }
  return 0;
}

static void create(const char *name, cell_t length, cell_t flags, void *op) {
  memcpy(g_sys.heap, name, length);  // name
  g_sys.heap += CELL_LEN(length);
  *g_sys.heap++ = length;  // length
  *g_sys.heap++ = (cell_t) g_sys.last;  // link
  *g_sys.heap++ = flags;  // flags
  g_sys.last = g_sys.heap;
  *g_sys.heap++ = (cell_t) op;  // code
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
  cell_t xt = find((const char *) name, len);
  if (xt) {
    if (g_sys.state && !(((cell_t *) xt)[-1] & 1)) {  // bit 0 of flags is immediate
      *g_sys.heap++ = xt;
    } else {
      call = xt;
    }
  } else {
    cell_t n;
    cell_t ok = convert((const char *) name, len, &n);
    if (ok) {
      if (g_sys.state) {
        *g_sys.heap++ = g_sys.DOLIT_XT;
        *g_sys.heap++ = n;
      } else {
        *++sp = n;
      }
    } else {
#if PRINT_ERRORS
      write(2, (void *) name, len);
#endif
      *++sp = name;
      *++sp = len;
      *++sp = -1;
      call = g_sys.notfound;
    }
  }
  *++sp = call;
  return sp;
}

static void ueforth(int argc, char *argv[], void *heap,
                    const char *src, cell_t src_len) {
  memset(&g_sys, 0, sizeof(g_sys));
  g_sys.heap = (cell_t *) heap;
  register cell_t *sp = g_sys.heap; g_sys.heap += STACK_SIZE;
  register cell_t *rp = g_sys.heap; g_sys.heap += STACK_SIZE;
  register cell_t tos = 0, *ip, w;
  dcell_t d;
  udcell_t ud;
#define X(name, op, code) create(name, sizeof(name) - 1, name[0] == ';', && OP_ ## op);
  PLATFORM_OPCODE_LIST
  OPCODE_LIST
#undef X
  g_sys.last[-1] = 1;  // Make ; IMMEDIATE
  g_sys.DOLIT_XT = FIND("DOLIT");
  g_sys.DOEXIT_XT = FIND("EXIT");
  g_sys.notfound = FIND("DROP");
  ip = g_sys.heap;
  *g_sys.heap++ = FIND("EVALUATE1");
  *g_sys.heap++ = FIND("BRANCH");
  *g_sys.heap++ = (cell_t) ip;
  g_sys.argc = argc;
  g_sys.argv = argv;
  g_sys.base = 10;
  g_sys.tib = src;
  g_sys.ntib = src_len;
  NEXT;
#define X(name, op, code) OP_ ## op: { code; } NEXT;
  PLATFORM_OPCODE_LIST
  OPCODE_LIST
#undef X
  OP_DOCOLON: ++rp; *rp = (cell_t) ip; ip = (cell_t *) (w + sizeof(cell_t)); NEXT;
  OP_DOCREATE: DUP; tos = w + sizeof(cell_t) * 2; NEXT;
  OP_DODOES: DUP; tos = w + sizeof(cell_t) * 2;
             ++rp; *rp = (cell_t) ip; ip = (cell_t *) *(cell_t *) (w + sizeof(cell_t)); NEXT;
}

const char boot[] =
" : (   41 parse drop drop ; immediate "

// Useful Basic Compound Words 
" : 2drop ( n n -- ) drop drop ; "
" : 2dup ( a b -- a b a b ) over over ; "
" : nip ( a b -- b ) swap drop ; "
" : rdrop ( r: n n -- ) r> r> drop >r ; "
" : */ ( n n n -- n ) */mod nip ; "
" : * ( n n -- n ) 1 */ ; "
" : /mod ( n n -- n n ) 1 swap */mod ; "
" : / ( n n -- n ) /mod nip ; "
" : mod ( n n -- n ) /mod drop ; "
" : invert ( n -- ~n ) -1 xor ; "
" : negate ( n -- -n ) invert 1 + ; "
" : - ( n n -- n ) negate + ; "
" : rot ( a b c -- c a b ) >r swap r> swap ; "
" : -rot ( a b c -- b c a ) swap >r swap r> ; "
" : < ( a b -- a<b ) - 0< ; "
" : > ( a b -- a>b ) swap - 0< ; "
" : = ( a b -- a!=b ) - 0= ; "
" : <> ( a b -- a!=b ) = 0= ; "
" : bl 32 ;   : nl 10 ; "
" : 1+ 1 + ;   : 1- 1 - ; "
" : 2* 2 * ;   : 2/ 2 / ; "
" : 4* 4 * ;   : 4/ 4 / ; "
" : +! ( n a -- ) swap over @ + swap ! ; "

// Cells 
" : cell+ ( n -- n ) cell + ; "
" : cells ( n -- n ) cell * ; "
" : cell/ ( n -- n ) cell / ; "

// System Variables 
" : 'tib ( -- a ) 'sys 0 cells + ; "
" : #tib ( -- a ) 'sys 1 cells + ; "
" : >in ( -- a ) 'sys 2 cells + ; "
" : state ( -- a ) 'sys 3 cells + ; "
" : base ( -- a ) 'sys 4 cells + ; "
" : 'heap ( -- a ) 'sys 5 cells + ; "
" : last ( -- a ) 'sys 6 cells + ; "
" : 'notfound ( -- a ) 'sys 7 cells + ; "
" : 'argc ( -- a ) 'sys 8 cells + ; "
" : 'argv ( -- a ) 'sys 9 cells + ; "

// Dictionary 
" : here ( -- a ) 'heap @ ; "
" : allot ( n -- ) 'heap +! ; "
" : aligned ( a -- a ) cell 1 - dup >r + r> invert and ; "
" : align   here aligned here - allot ; "
" : , ( n --  ) here ! cell allot ; "
" : c, ( ch -- ) here c! 1 allot ; "

// Compilation State 
" : [ 0 state ! ; immediate "
" : ] -1 state ! ; immediate "

// Quoting Words 
" : ' bl parse 2dup find dup >r -rot r> 0= 'notfound @ execute 2drop ; "
" : ['] ' aliteral ; immediate "
" : char bl parse drop c@ ; "
" : [char] char aliteral ; immediate "
" : literal aliteral ; immediate "

// Core Control Flow 
" : begin   here ; immediate "
" : again   ['] branch , , ; immediate "
" : until   ['] 0branch , , ; immediate "
" : ahead   ['] branch , here 0 , ; immediate "
" : then   here swap ! ; immediate "
" : if   ['] 0branch , here 0 , ; immediate "
" : else   ['] branch , here 0 , swap here swap ! ; immediate "
" : while   ['] 0branch , here 0 , swap ; immediate "
" : repeat   ['] branch , , here swap ! ; immediate "
" : aft   drop ['] branch , here 0 , here swap ; immediate "

// Compound words requiring conditionals 
" : min 2dup < if drop else nip then ; "
" : max 2dup < if nip else drop then ; "
" : abs ( n -- +n ) dup 0< if negate then ; "

// Dictionary Format 
" : >name ( xt -- a n ) 3 cells - dup @ swap over aligned - swap ; "
" : >link ( xt -- a ) 2 cells - @ ; "
" : >flags ( xt -- flags ) cell - ; "
" : >body ( xt -- a ) 2 cells + ; "
" : >:body ( xt -- a ) cell+ ; "

// Postpone - done here so we have ['] and IF 
" : immediate? ( xt -- f ) >flags @ 1 and 0= 0= ; "
" : postpone ' dup immediate? if , else aliteral ['] , , then ; immediate "

// Counted Loops 
" : for   postpone >r postpone begin ; immediate "
" : next   postpone donext , ; immediate "
" : do   postpone swap postpone >r postpone >r here ; immediate "
" : i   postpone r@ ; immediate "
" : j   rp@ 3 cells - @ ; "
" : unloop   postpone rdrop postpone rdrop ; immediate "
" : +loop   postpone r> postpone + postpone r> "
"           postpone 2dup postpone >r postpone >r "
"           postpone < postpone 0= postpone until "
"           postpone unloop ; immediate "
" : loop   1 aliteral postpone +loop ; immediate "

// Constants and Variables 
" : constant ( n \"name\" -- ) create , does> @ ; "
" : variable ( \"name\" -- ) create 0 , ; "

// Stack Convience 
" sp@ constant sp0 "
" rp@ constant rp0 "
" : depth ( -- n ) sp@ sp0 - cell/ ; "

// Exceptions 
" variable handler "
" : catch ( xt -- n ) "
"   sp@ >r handler @ >r rp@ handler ! execute r> handler ! r> drop 0 ; "
" : throw ( n -- ) "
"   dup if handler @ rp! r> handler !  r> swap >r sp! drop r> else drop then ; "
" ' throw 'notfound ! "

// Values 
" : value ( n -- ) create , does> @ ; "
" : to ( n -- ) state @ if postpone ['] postpone >body postpone ! "
"                       else ' >body ! then ; immediate "

// Deferred Words 
" : defer ( \"name\" -- ) create 0 , does> @ dup 0= throw execute ; "
" : is ( xt \"name -- ) postpone to ; immediate "

// Defer I/O to platform specific 
" defer type "
" defer key "
" defer bye "
" : emit ( n -- ) >r rp@ 1 type rdrop ; "
" : space bl emit ;   : cr nl emit ; "

// Numeric Output 
" variable hld "
" : pad ( -- a ) here 80 + ; "
" : digit ( u -- c ) 9 over < 7 and + 48 + ; "
" : extract ( n base -- n c ) 0 swap um/mod swap digit ; "
" : <# ( -- ) pad hld ! ; "
" : hold ( c -- ) hld @ 1 - dup hld ! c! ; "
" : # ( u -- u ) base @ extract hold ; "
" : #s ( u -- 0 ) begin # dup while repeat ; "
" : sign ( n -- ) 0< if 45 hold then ; "
" : #> ( w -- b u ) drop hld @ pad over - ; "
" : str ( n -- b u ) dup >r abs <# #s r> sign #> ; "
" : hex ( -- ) 16 base ! ;   : octal ( -- ) 8 base ! ; "
" : decimal ( -- ) 10 base ! ; "
" : u. ( u -- ) <# #s #> type space ; "
" : . ( w -- ) base @ 10 xor if u. exit then str type space ; "
" : ? ( a -- ) @ . ; "

// Strings 
" : parse-quote ( -- a n ) [char] \" parse ; "
" : $place ( a n -- ) 0 do dup c@ c, 1+ loop drop 0 c, align ; "
" : $@   r@ dup cell+ swap @ r> dup @ 1+ aligned + cell+ >r ; "
" : s\"   parse-quote state @ if postpone $@ dup , $place "
"        else dup here swap >r >r $place r> r> then ; immediate "
" : .\"   postpone s\" state @ if postpone type else type then ; immediate "
" : z\"   postpone s\" state @ if postpone drop else drop then ; immediate "
" : r\"   parse-quote state @ if swap aliteral aliteral then ; immediate "

// Better Errors 
" : notfound ( a n n -- ) "
"    if cr .\" ERROR: \" type .\"  NOT FOUND!\" cr -1 throw then ; "
" ' notfound 'notfound ! "

// Examine Dictionary 
" : see. ( xt -- ) >name type space ; "
" : see-one ( xt -- xt+1 ) "
"    dup @ dup ['] DOLIT = if drop cell+ dup @ . else see. then cell+ ; "
" : exit= ( xt -- ) ['] exit = ; "
" : see-loop   >:body begin see-one dup @ exit= until ; "
" : see   cr ['] : see.  ' dup see.  space see-loop drop  ['] ; see.  cr ; "
" : words   last @ begin dup see. >link dup 0= until drop cr ; "

// Examine Memory 
" : dump ( a n -- ) "
"    cr 0 do i 16 mod 0= if cr then dup i + c@ . loop drop cr ; "

// Input 
" : accept ( a n -- n ) 0 swap begin 2dup < while "
"    key dup nl = if 2drop nip exit then "
"    >r rot r> over c! 1+ -rot swap 1+ swap repeat drop nip ; "
" 200 constant input-limit "
" : tib ( -- a ) 'tib @ ; "
" create input-buffer   input-limit allot "
" : tib-setup   input-buffer 'tib ! ; "
" : refill   tib-setup tib input-limit accept #tib ! 0 >in ! -1 ; "

// REPL 
" : prompt   .\"  ok\" cr ; "
" : evaluate-buffer   begin >in @ #tib @ < while evaluate1 repeat ; "
" : evaluate ( a n -- ) 'tib @ >r #tib @ >r >in @ >r "
"                       #tib ! 'tib ! 0 >in ! evaluate-buffer "
"                       r> >in ! r> #tib ! r> 'tib ! ; "
" : query   begin ['] evaluate-buffer catch "
"           if 0 state ! sp0 sp! rp0 rp! .\" ERROR\" cr then "
"           prompt refill drop again ; "
" : ok   .\" uEForth\" cr prompt refill drop query ; "
// Set up Basic I/O 
" : arduino-bye   0 terminate ; "
" ' arduino-bye is bye "
" : arduino-type ( a n -- ) Serial.write drop ; "
" ' arduino-type is type "
" : key? ( -- n ) Serial.available ; "
" : arduino-key ( -- n ) "
"    begin Serial.available until 0 >r rp@ 1 Serial.readBytes drop r> ; "
" ' arduino-key is key "

// Map Arduino / ESP32 things to shorter names. 
" : pin ( n n -- ) swap digitalWrite ; "
" : adc ( n -- n ) analogRead ; "
" : duty ( n n -- ) 255 min 8191 255 */ ledcWrite ; "
" : freq ( n n -- ) 1000 * 13 ledcSetup drop ; "
" : tone ( n n -- ) 1000 * ledcWriteTone drop ; "

// Startup Serial 
" 115200 Serial.begin "
" 100 ms "
" ok "
;


static IPAddress ToIP(cell_t ip) {
  return IPAddress(ip & 0xff, ((ip >> 8) & 0xff), ((ip >> 16) & 0xff), ((ip >> 24) & 0xff));
}

static cell_t FromIP(IPAddress ip) {
  cell_t ret = 0;
  ret = (ret << 8) | ip[3];
  ret = (ret << 8) | ip[2];
  ret = (ret << 8) | ip[1];
  ret = (ret << 8) | ip[0];
  return ret;
}

void setup() {
  cell_t *heap = (cell_t *) malloc(HEAP_SIZE);
  ueforth(0, 0, heap, boot, sizeof(boot));
}

void loop() {
}
