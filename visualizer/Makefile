#!/bin/make
OF_ROOT=${CURDIR}/../third_party/openFrameworks
OF_INCLUDE =$(OF_ROOT)/libs/openFrameworksCompiled/project/makefileCommon/compile.project.mk
include $(OF_INCLUDE)

.PHONY: build
build:
	$(MAKE) PLATFORM_CFLAGS=-D_MAPDIR_='\"${CURDIR}/../map/\"'

.PHONY: ofrun
ofrun:
	@./bin/$(APPNAME).app/Contents/MacOS/$(APPNAME) ${file}
