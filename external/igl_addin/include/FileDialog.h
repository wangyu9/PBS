// IGL Viewer - Copyright (c) 2013 ETH Zurich. All rights reserved.
// This file contains cross-platform functions to open load/save file dialogs

#ifndef FileDialog
#define FileDialog

#include <stdio.h>
#define FILE_DIALOG_MAX_BUFFER 1024

#ifdef _WIN32
 #include <Commdlg.h>
#endif

// Sets buffer to a path to an existing file
// buffer[0]=0 on cancel
//
// Usage:
//   char buffer[FILE_DIALOG_MAX_BUFFER];
//   get_open_file_path(buffer);
void inline get_open_file_path(char buffer[])
{
  bool filter = false;
  char command[128];
  
#ifdef __APPLE__
  // For apple use applescript hack
  FILE * output = popen(
                        "osascript -e \""
                        "   tell application \\\"System Events\\\"\n"
                        "           activate\n"
                        "           set existing_file to choose file\n"
                        "   end tell\n"
                        "   set existing_file_path to (POSIX path of (existing_file))\n"
                        "\" 2>/dev/null | tr -d '\n' ","r");
  while ( fgets(buffer, FILE_DIALOG_MAX_BUFFER, output) != NULL ){
  }
#elif _WIN32
  
  /*// For windows use Zenity (http://www.placella.com/software/zenity/)
  if (filter)
    sprintf(command, "zenity --file-selection --file-filter=\"%s\"", filter);
  else
    sprintf(command, "zenity --file-selection");
  FILE * output = _popen(command,"r");
  while ( fgets(buffer, FILE_DIALOG_MAX_BUFFER, output) != NULL ){
  }
  
  if (strlen(buffer) > 0)
    buffer[strlen(buffer)-1] = 0;*/

	// Use native windows file dialog box instead of Zenity
	OPENFILENAME ofn;       // common dialog box structure
	char szFile[260];       // buffer for file name
	HWND hwnd;              // owner window
	HANDLE hf;              // file handle

	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;//hwnd;
	ofn.lpstrFile = new wchar_t[100];
	// Set lpstrFile[0] to '\0' so that GetOpenFileName does not 
	// use the contents of szFile to initialize itself.
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = sizeof(szFile);
	ofn.lpstrFilter = L"*.*\0";//off\0*.off\0obj\0*.obj\0mp\0*.mp\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = NULL;
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	// Display the Open dialog box. 
	int pos = 0;
	if (GetOpenFileName(&ofn)==TRUE)
	{
		while(ofn.lpstrFile[pos] != '\0')
		{
			buffer[pos] = (char)ofn.lpstrFile[pos];
			pos++;
		}
		buffer[pos] = 0;
	}	
  
#else
  // For linux use zenity
  if (filter)
    sprintf(command, "/usr/bin/zenity --file-selection --file-filter=\"%s\"", filter);
  else
    sprintf(command, "/usr/bin/zenity --file-selection");
  FILE * output = popen(command,"r");
  while ( fgets(buffer, FILE_DIALOG_MAX_BUFFER, output) != NULL ){
  }
  
  if (strlen(buffer) > 0)
    buffer[strlen(buffer)-1] = 0;
#endif
}

// Sets buffer to a path to a new/existing file
// buffer[0]=0 on cancel
//
// Usage:
//   char buffer[FILE_DIALOG_MAX_BUFFER];
//   get_save_file_path(buffer);
void inline get_save_file_path(char buffer[]){
  bool filter = false;

#ifdef __APPLE__
  // For apple use applescript hack
  // There is currently a bug in Applescript that strips extensions off
  // of chosen existing files in the "choose file name" dialog
  // I'm assuming that will be fixed soon
  FILE * output = popen(
                        "osascript -e \""
                        "   tell application \\\"System Events\\\"\n"
                        "           activate\n"
                        "           set existing_file to choose file name\n"
                        "   end tell\n"
                        "   set existing_file_path to (POSIX path of (existing_file))\n"
                        "\" 2>/dev/null | tr -d '\n' ","r");
  while ( fgets(buffer, FILE_DIALOG_MAX_BUFFER, output) != NULL ){
  }
#elif _WIN32
  /*// For windows use Zenity (http://www.placella.com/software/zenity/)
  char command[128];
  if (filter)
    sprintf(command, "zenity --file-selection --save --file-filter=\"%s\"", filter);
  else
    sprintf(command, "zenity --file-selection --save");
  FILE * output = _popen(command,"r");
  while ( fgets(buffer, FILE_DIALOG_MAX_BUFFER, output) != NULL ){
  }
  
  if (strlen(buffer) > 0)
    buffer[strlen(buffer)-1] = 0;*/

  // Use native windows file dialog box instead of Zenity
	OPENFILENAME ofn;       // common dialog box structure
	char szFile[260];       // buffer for file name
	HWND hwnd;              // owner window
	HANDLE hf;              // file handle

	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;//hwnd;
	ofn.lpstrFile = new wchar_t[100];
	// Set lpstrFile[0] to '\0' so that GetOpenFileName does not 
	// use the contents of szFile to initialize itself.
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = sizeof(szFile);
	ofn.lpstrFilter = L"";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = NULL;
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	// Display the Open dialog box. 
	int pos = 0;
	if (GetSaveFileName(&ofn)==TRUE)
	{
		while(ofn.lpstrFile[pos] != '\0')
		{
			buffer[pos] = (char)ofn.lpstrFile[pos];
			pos++;
		}
		buffer[pos] = 0;
	}

#else
  // For every other machine type use zenity
  char command[128];
  if (filter)
    sprintf(command, "/usr/bin/zenity --file-selection --save --file-filter=\"%s\"", filter);
  else
    sprintf(command, "/usr/bin/zenity --file-selection --save");
  FILE * output = popen(command,"r");
  while ( fgets(buffer, FILE_DIALOG_MAX_BUFFER, output) != NULL ){
  }
  
  if (strlen(buffer) > 0)
    buffer[strlen(buffer)-1] = 0;
#endif
}

#endif