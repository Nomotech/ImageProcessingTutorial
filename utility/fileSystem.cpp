#include <Windows.h>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <string>
#include "fileSystem.hpp"

// 入力フォルダ名の指定

// INPUT_FOLDER_NAMEのフォルダ内にある画像名を取得する
std::vector<std::string> getImageList(std::string path) {

	HANDLE hFind;
	WIN32_FIND_DATA win32fd;
	std::vector<std::string> file_names;

	// png,jpg,bmpの拡張子のファイルのみを読み込む
	std::string extension[3] = {"png" ,"jpg", "bmp"};

	for (int i = 0; i < 3; i++) {

		std::string search_name = path + "*." + extension[i];
		hFind = FindFirstFile(search_name.c_str(), &win32fd);

		if (hFind == INVALID_HANDLE_VALUE) {
			continue;
		}
		do {
			if (win32fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			} else {
				file_names.push_back(win32fd.cFileName);
			}
		} while (FindNextFile(hFind, &win32fd));

		FindClose(hFind);
	}
	return file_names;
}

