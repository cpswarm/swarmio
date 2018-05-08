REM Get working directory
set cwd=%cd%

REM Build dependencies
mkdir dependencies
cd dependencies
cmake ..\..\dependencies -DCMAKE_INSTALL_PREFIX=%cwd%\dependencies-prefix
cmake --build .
cd ..

REM Build components
mkdir main
cd main
cmake ..\.. -DCMAKE_PREFIX_PATH=%cwd%\dependencies-prefix
cmake --build .

