AUTOMAKE_OPTIONS = subdir-objects
ACLOCAL_AMFLAGS = ${ACLOCAL_FLAGS}

bin_PROGRAMS = stringformatter
stringformatter_SOURCES = src/main.cpp
stringformatter_CPPFLAGS = $(fmt_CFLAGS)
stringformatter_LDADD = $(fmt_LIBS)
