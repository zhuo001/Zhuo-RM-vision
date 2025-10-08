
.PHONY:: all 
MAKE := make

SUBDIRS = HawkColor HawkDepth HawkIr HawkLightIr HawkMixColorDepth HawkMixHDColorDepth HawkColorDoubleDevice HawkDepthDoubleDevice HawkMixColorDepthDoubleDevice HawkMixColorDepthMatting HawkNetDeviceInfo

all::$(SUBDIRS)
	@for f in $^; do (cd $$f&&$(MAKE) --file=Makefile&&cd -)||exit; done

clean::$(SUBDIRS)
	@for f in $^; do (cd $$f&&$(MAKE) --file=Makefile clean&&cd -)||exit; done
