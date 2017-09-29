@echo off
REM This batch file takes the project folder (e.g. ...\trunk\monolith or ...\rev61\TestICal) as a parameter (%1)
REM NOTE: SubWCRev.exe is assumed to be in C:\Program Files\TortoiseSVN\bin
REM First: SVN revision for the project folder
"%SystemDrive%\Program Files"\TortoiseSVN\bin\SubWCRev.exe %1 %1\..\common\GetSVNrev_in.h %1\..\common\SVNrev.h
REM Next: SVN revision for the "common" folder
"%SystemDrive%\Program Files"\TortoiseSVN\bin\SubWCRev.exe %1\..\common %1\..\common\GetComRev_in.h %1\..\common\ComRev.h
REM Next: SVN revision specifically for BSL2.s43 (since it can be troublesome)
"%SystemDrive%\Program Files"\TortoiseSVN\bin\SubWCRev.exe %1\..\common\BSL2.s43 %1\..\common\GetBSLrev_in.h %1\..\common\BSLrev.h
