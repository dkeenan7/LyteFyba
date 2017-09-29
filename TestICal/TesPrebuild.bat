@echo off
REM This batch file does the prebuild actions for the TestICal project
REM It takes the project folder (e.g. ...\trunk\TestICal) as a parameter (%1)
REM First create the file BSL2Macro.s43 by wrapping a macro definition around BSL2.s43
REM This is necessary because IAR will not allow #includes inside macro definitions
type %1\BSL2Macro.h %1\..\common\BSL2.s43 %1\BSL2Endm.h > %1\BSL2Macro.s43
REM Now pass control to the batch file that creates the files with the SVN revision numbers
%1\..\common\GetSVNrevs.bat %1
