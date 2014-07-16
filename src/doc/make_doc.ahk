; AutoHotkey Script (see also http://www.autohotkey.com)

#NoEnv                          ; Recommended for performance and compatibility with future AutoHotkey releases.
SendMode Input                  ; Recommended for new scripts due to its superior speed and reliability.
SetWorkingDir %A_ScriptDir%     ; Ensures a consistent starting directory.

^!d:: RunWait make_doc.sh       ; Set global hotkey CTRL + ALT + D to run "make_doc.sh"