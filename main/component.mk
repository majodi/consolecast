#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_EMBED_TXTFILES := certs/cacert.pem
COMPONENT_EMBED_TXTFILES += certs/prvtkey.pem
COMPONENT_EMBED_TXTFILES += html/index.html
COMPONENT_EMBED_TXTFILES += html/favicon.ico
COMPONENT_EMBED_TXTFILES += html/cc.png
COMPONENT_EMBED_TXTFILES += html/xterm.js
