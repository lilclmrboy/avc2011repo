# software/examples/aRFiCB2App makefile

ARCH := `arch`
OS := $(shell uname)

APP = aHeadlessChickenApp

.PHONY : $(APP)
.PHONY : clean

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# Mac build commands
ifeq ($(OS), Darwin)
$(APP) :
	xcodebuild -target $@ -configuration aBinary
clean :
	xcodebuild -alltargets -configuration aBinary clean
	rm -rfv macx_$(APP)_Data
endif


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# Linux build commands
ifeq ($(OS),Linux)
$(APP) :
	make -f unix_makefile app
clean :
	make -f unix_makefile clean
endif


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# Windows build commands
ifeq ($(OS),WindowsNT)
$(APP) :
	MSBuild /property:Configuration=Debug win32_$(APP).vcxproj
	MSBuild /property:Configuration=Release win32_$(APP).vcxproj
clean :
	MSBuild /property:Configuration=Debug /target:clean win32_$(APP).vcxproj
	MSBuild /property:Configuration=Release /target:clean win32_$(APP).vcxproj
	rm -rf win32_$(APP)_Data
	rm -f win32_$(APP).ncb
	rm -f win32_$(APP).sln
	rm -f win32_$(APP).sdf
	rm -f win32_$(APP).suo
	rm -f win32_$(APP).vcxproj.user
	rm -f win32_$(APP).vcxproj.filters
endif
