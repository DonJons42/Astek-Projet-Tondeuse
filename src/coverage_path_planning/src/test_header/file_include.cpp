#include <stdio.h>
#include <stdlib.h>
#include "file_include.h"

using namespace std;

Nombre_test::Nombre_test(int a, float b){
	nombre = a;
	virgule = b;
}

void fonction_include(Nombre_test bidule){
cout << "FONCTION INCLUDE :" << bidule.nombre << " " << bidule.virgule << endl;
}
