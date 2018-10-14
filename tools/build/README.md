### Build support files for Windows 

copy `srec_cat.exe` and `mingw.exe` to c:\Program Files\Git\mingw64\bin\.

Then invoke builds with, for example: 

`"c:\Program Files\Git\bin\bash.exe" -c "gmake TARGET=simplelink BOARD=launchpad/cc1352r1"`

git-bash is required (instead of just a regular Windows console) due to some 'linuxy' features added to the newer simplelink makefiles: namely the find command which won't work with Windows' find.
