WINELIB = /usr/lib64/wine
WINEINC = /usr/include/wine

WINE_INCLUDES = -I$(WINEINC)/windows -I$(WINEINC)/msvcrt

CC = clang
CFLAGS = \
    -m64 \
    -O2 \
    -D__WINESRC__ \
    -DWINE_UNIX_LIB \
    -D_WIN64 \
    -pipe \
    -fcf-protection=none \
    -fvisibility=hidden \
    -fno-stack-protector \
    -fno-strict-aliasing \
    -fPIC \
    -fasynchronous-unwind-tables \
    -Wall \
    -Wdeclaration-after-statement \
    -Wempty-body \
    -Wignored-qualifiers \
    -Winit-self \
    -Wno-pragma-pack \
    -Wstrict-prototypes \
    -Wtype-limits \
    -Wunused-but-set-parameter \
    -Wvla \
    -Wwrite-strings \
    -Wpointer-arith
LDFLAGS = \
    -shared \
    -Wl,-Bsymbolic \
    -Wl,-z,defs \
    -L$(WINELIB)/x86_64-unix -l:ntdll.so

WIN_LIBS = -lwinecrt0 -lucrtbase -lkernel32 -lntdll

i386_CC = clang
i386_CFLAGS = \
    -O2 \
    -D__STDC__ \
    -D__WINE_PE_BUILD \
    -D_UCRT \
    -D__WINESRC__ \
    -target i686-windows \
    -fuse-ld=lld \
    --no-default-config \
    -fno-strict-aliasing \
    -fno-omit-frame-pointer \
    -Wall \
    -Wdeclaration-after-statement \
    -Wempty-body \
    -Wignored-qualifiers \
    -Winit-self \
    -Wno-pragma-pack \
    -Wno-microsoft-enum-forward-reference \
    -Wstrict-prototypes \
    -Wtype-limits \
    -Wunused-but-set-parameter \
    -Wvla \
    -Wwrite-strings \
    -Wpointer-arith \
    -Wabsolute-value \
    -Wenum-conversion \
    $(WINE_INCLUDES)
i386_LDFLAGS = \
    -b i686-windows \
    -Wl,--wine-builtin,/safeseh:no \
    -shared \
    --no-default-config \
    -L$(WINELIB)/i386-windows $(WIN_LIBS)
i386_DIR = i386-windows

x86_64_CC = clang
x86_64_CFLAGS = \
    -O2 \
    -D__STDC__ \
    -D__WINE_PE_BUILD \
    -D_UCRT \
    -D__WINESRC__ \
    -target x86_64-windows \
    -fuse-ld=lld \
    --no-default-config \
    -fno-strict-aliasing \
    -Wall \
    -Wdeclaration-after-statement \
    -Wempty-body \
    -Wignored-qualifiers \
    -Winit-self \
    -Wno-pragma-pack \
    -Wno-microsoft-enum-forward-reference \
    -Wstrict-prototypes \
    -Wtype-limits \
    -Wunused-but-set-parameter \
    -Wvla \
    -Wwrite-strings \
    -Wpointer-arith \
    -Wabsolute-value \
    -Wenum-conversion \
    -Wformat-overflow \
    -Wnonnull \
    -mcx16 \
    -mcmodel=small \
    $(WINE_INCLUDES)
x86_64_LDFLAGS = \
    -b x86_64-windows \
    -Wl,--wine-builtin \
    -shared \
    --no-default-config \
    -L$(WINELIB)/x86_64-windows $(WIN_LIBS)
x86_64_DIR = x86_64-windows

SRCS = \
    j2534drv.c \
    utils.c \
    j2534translate.c \
    log.c

all: i386-windows/j2534drv.dll x86_64-windows/j2534drv.dll

$(i386_DIR) $(x86_64_DIR):
	mkdir -p $@


$(i386_DIR)/%.o: %.c | $(i386_DIR)
	$(i386_CC) -c -o $@ $< $(i386_CFLAGS)

$(x86_64_DIR)/%.o: %.c | $(x86_64_DIR)
	$(x86_64_CC) -c -o $@ $< $(x86_64_CFLAGS)

j2534drv.a: j2534drv.spec
	winebuild -w --implib -o $@ -m64 --export $^

$(i386_DIR)/j2534drv.a: j2534drv.spec
	winebuild -w --implib -o $@ -b i686-windows --export $^

$(x86_64_DIR)/j2534drv.a: j2534drv.spec
	winebuild -w --implib -o $@ --without-dlltool -b x86_64-w64-mingw32 --export $^


$(i386_DIR)/j2534drv.dll: j2534drv.spec $(addprefix $(i386_DIR)/, $(SRCS:.c=.o))
	winegcc -o $@ $^ $(i386_LDFLAGS)

$(x86_64_DIR)/j2534drv.dll: j2534drv.spec $(addprefix $(x86_64_DIR)/, $(SRCS:.c=.o))
	winegcc -o $@ $^ $(x86_64_LDFLAGS)

install install-lib:: i386-windows/j2534drv.dll x86_64-windows/j2534drv.dll 
	install -m 644 $(INSTALL_PROGRAM_FLAGS) i386-windows/j2534drv.dll $(DESTDIR)$(WINELIB)/i386-windows/j2534drv.dll
	winebuild --builtin $(DESTDIR)$(WINELIB)/i386-windows/j2534drv.dll
	install -m 644 $(INSTALL_PROGRAM_FLAGS) x86_64-windows/j2534drv.dll $(DESTDIR)$(WINELIB)/x86_64-windows/j2534drv.dll
	winebuild --builtin $(DESTDIR)$(WINELIB)/x86_64-windows/j2534drv.dll

install install-dev:: j2534drv.a i386-windows/j2534drv.a x86_64-windows/j2534drv.a
	install -m 644 $(INSTALL_DATA_FLAGS) j2534drv.a $(DESTDIR)$(WINELIB)/x86_64-unix/j2534drv.a
	install -m 644 $(INSTALL_DATA_FLAGS) i386-windows/j2534drv.a $(DESTDIR)$(WINELIB)/i386-windows/j2534drv.a
	install -m 644 $(INSTALL_DATA_FLAGS) x86_64-windows/j2534drv.a $(DESTDIR)$(WINELIB)/x86_64-windows/j2534drv.a

uninstall uninstall-lib::
	rm -f $(DESTDIR)$(WINELIB)/i386-windows/j2534drv.dll
	rm -f $(DESTDIR)$(WINELIB)/x86_64-windows/j2534drv.dll

uninstall uninstall-dev::
	rm -f $(DESTDIR)$(WINELIB)/i386-windows/j2534drv.a
	rm -f $(DESTDIR)$(WINELIB)/x86_64-windows/j2534drv.a

clean::
	rm -f j2534drv.a
	rm -rf $(i386_DIR) $(x86_64_DIR)
