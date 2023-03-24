#!/bin/sh

set -ex

aclocal \
&& automake --add-missing \
&& autoconf
