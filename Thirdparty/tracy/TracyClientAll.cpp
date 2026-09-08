// The whole Tracy client is one amalgamated translation unit. It is vendored
// verbatim and tracks upstream, so its warnings are suppressed here -- around
// the include, not inside the vendored sources -- rather than by widening the
// package-wide -Wall -Wextra that the rest of the tree is clean under.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-result"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wempty-body"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wformat-overflow"
#include "public/TracyClient.cpp"
#pragma GCC diagnostic pop
