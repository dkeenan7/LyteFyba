@echo off
REM This batch file runs from the trunk or rev61 or rev58 folder
REM NOTE: SubWCRev.exe must be in the path; usually c:\Program Files\TortoiseSVN\bin
REM First: SVN version for the whole repository branch/trunk
SubWCRev.exe . common\GetSVNrev_in.h common\SVNrev.h
REM Next: SVN version specifically for BSL2.s43 (since it can be troublesome)
SubWCRev.exe common\BSL2.s43 common\GetBSLrev_in.h common\BSLrev.h

