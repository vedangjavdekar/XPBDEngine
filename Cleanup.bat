@echo off
IF EXIST .vs (
    RMDIR /S /Q .vs
)

IF EXIST .\build (
    RMDIR /S /Q .\build
)

IF EXIST .\bin (
    RMDIR /S /Q .\bin
)

IF EXIST .\bin-int (
    RMDIR /S /Q .\bin-int
)

IF EXIST .\*.sln (
    DEL /S /Q .\*.sln
)