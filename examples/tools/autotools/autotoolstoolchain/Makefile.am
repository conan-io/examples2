AUTOMAKE_OPTIONS = subdir-objects
ACLOCAL_AMFLAGS = ${ACLOCAL_FLAGS}

bin_PROGRAMS = stringformatter
stringformatter_SOURCES = src/main.cpp

clean-local:
	@rm -rfconfig.status configure config.log
	@rm -rf Makefile
	@rm -rf autom4te.cache/
	@rm -rf aclocal.m4 depcomp
	@rm -rf compile install-sh missing Makefile.in
