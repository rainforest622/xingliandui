@echo off
set "CCACHE_TARGET=%~1"
shift
"%CCACHE_TARGET%" %*
