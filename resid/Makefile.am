## Process this file with automake to create Makefile.in

ACLOCAL_AMFLAGS = --install -I m4

AR = @AR@

AM_CXXFLAGS = -std=gnu++14  # Enforce C++14 standard

noinst_LIBRARIES = libresid.a

libresid_a_SOURCES = sid.cc voice.cc wave.cc envelope.cc filter.cc extfilt.cc pot.cc version.cc

BUILT_SOURCES = $(noinst_DATA:.dat=.h)

noinst_HEADERS = sid.h voice.h wave.h envelope.h filter.h dac.h extfilt.h pot.h spline.h $(noinst_DATA:.dat=.h)

noinst_DATA = wave6581_PST.dat wave6581_PS_.dat wave6581_P_T.dat wave6581__ST.dat wave8580_PST.dat wave8580_PS_.dat wave8580_P_T.dat wave8580__ST.dat

noinst_SCRIPTS = samp2src.pl

EXTRA_DIST = $(noinst_HEADERS) $(noinst_DATA) $(noinst_SCRIPTS)

SUFFIXES = .dat

.dat.h:
	$(PERL) $(srcdir)/samp2src.pl $* $< $(srcdir)/$@
