#!/bin/make
OF_INCLUDE = $(OF_ROOT)/libs/openFrameworksCompiled/project/makefileCommon/compile.project.mk

ifdef OF_ROOT
	include $(OF_INCLUDE)
endif

.PHONY: ofrun
ofrun:
	@./bin/$(APPNAME).app/Contents/MacOS/$(APPNAME) ${file}
