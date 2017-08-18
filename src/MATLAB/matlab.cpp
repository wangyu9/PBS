#include "matlab.h"


int creat_main() {
  MATFile *pmat;
  mxArray *pa1, *pa2, *pa3;
  double data[9] = { 1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0 };
  const char *file = "mattest.mat";
  char str[BUFSIZE];
  int status; 

  printf("Creating file %s...\n\n", file);
  pmat = matOpen(file, "w");
  if (pmat == NULL) {
    printf("Error creating file %s\n", file);
    printf("(Do you have write permission in this directory?)\n");
    return(EXIT_FAILURE);
  }

  pa1 = mxCreateDoubleMatrix(3,3,mxREAL);
  if (pa1 == NULL) {
      printf("%s : Out of memory on line %d\n", __FILE__, __LINE__); 
      printf("Unable to create mxArray.\n");
      return(EXIT_FAILURE);
  }

  pa2 = mxCreateDoubleMatrix(3,3,mxREAL);
  if (pa2 == NULL) {
      printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
      printf("Unable to create mxArray.\n");
      return(EXIT_FAILURE);
  }
  memcpy((void *)(mxGetPr(pa2)), (void *)data, sizeof(data));
  
  pa3 = mxCreateString("MATLAB: the language of technical computing");
  if (pa3 == NULL) {
      printf("%s :  Out of memory on line %d\n", __FILE__, __LINE__);
      printf("Unable to create string mxArray.\n");
      return(EXIT_FAILURE);
  }

  status = matPutVariable(pmat, "LocalDouble", pa1);
  if (status != 0) {
      printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
      return(EXIT_FAILURE);
  }  
  
  status = matPutVariableAsGlobal(pmat, "GlobalDouble", pa2);
  if (status != 0) {
      printf("Error using matPutVariableAsGlobal\n");
      return(EXIT_FAILURE);
  } 
  
  status = matPutVariable(pmat, "LocalString", pa3);
  if (status != 0) {
      printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
      return(EXIT_FAILURE);
  } 
  
  /*
   * Ooops! we need to copy data before writing the array.  (Well,
   * ok, this was really intentional.) This demonstrates that
   * matPutVariable will overwrite an existing array in a MAT-file.
   */
  memcpy((void *)(mxGetPr(pa1)), (void *)data, sizeof(data));
  status = matPutVariable(pmat, "LocalDouble", pa1);
  if (status != 0) {
      printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
      return(EXIT_FAILURE);
  } 
  
  /* clean up */
  mxDestroyArray(pa1);
  mxDestroyArray(pa2);
  mxDestroyArray(pa3);

  if (matClose(pmat) != 0) {
    printf("Error closing file %s\n",file);
    return(EXIT_FAILURE);
  }

  /*
   * Re-open file and verify its contents with matGetVariable
   */
  pmat = matOpen(file, "r");
  if (pmat == NULL) {
    printf("Error reopening file %s\n", file);
    return(EXIT_FAILURE);
  }

  /*
   * Read in each array we just wrote
   */
  pa1 = matGetVariable(pmat, "LocalDouble");
  if (pa1 == NULL) {
    printf("Error reading existing matrix LocalDouble\n");
    return(EXIT_FAILURE);
  }
  if (mxGetNumberOfDimensions(pa1) != 2) {
    printf("Error saving matrix: result does not have two dimensions\n");
    return(EXIT_FAILURE);
  }

  pa2 = matGetVariable(pmat, "GlobalDouble");
  if (pa2 == NULL) {
    printf("Error reading existing matrix GlobalDouble\n");
    return(EXIT_FAILURE);
  }
  if (!(mxIsFromGlobalWS(pa2))) {
    printf("Error saving global matrix: result is not global\n");
    return(EXIT_FAILURE);
  }

  pa3 = matGetVariable(pmat, "LocalString");
  if (pa3 == NULL) {
    printf("Error reading existing matrix LocalString\n");
    return(EXIT_FAILURE);
  }
  
  status = mxGetString(pa3, str, sizeof(str));
  if(status != 0) {
      printf("Not enough space. String is truncated.");
      return(EXIT_FAILURE);
  }
  if (strcmp(str, "MATLAB: the language of technical computing")) {
    printf("Error saving string: result has incorrect contents\n");
    return(EXIT_FAILURE);
  }

  /* clean up before exit */
  mxDestroyArray(pa1);
  mxDestroyArray(pa2);
  mxDestroyArray(pa3);

  if (matClose(pmat) != 0) {
    printf("Error closing file %s\n",file);
    return(EXIT_FAILURE);
  }
  printf("Done\n");
  return(EXIT_SUCCESS);
}


int diagnose(const char *file) {
  MATFile *pmat;
  const char **dir;
  const char *name;
  int	  ndir;
  int	  i;
  mxArray *pa;
  
  printf("Reading file %s...\n\n", file);

  /*
   * Open file to get directory
   */
  pmat = matOpen(file, "r");
  if (pmat == NULL) {
    printf("Error opening file %s\n", file);
    return(1);
  }
  
  /*
   * get directory of MAT-file
   */
  dir = (const char **)matGetDir(pmat, &ndir);
  if (dir == NULL) {
    printf("Error reading directory of file %s\n", file);
    return(1);
  } else {
    printf("Directory of %s:\n", file);
    for (i=0; i < ndir; i++)
      printf("%s\n",dir[i]);
  }
  mxFree(dir);

  /* In order to use matGetNextXXX correctly, reopen file to read in headers. */
  if (matClose(pmat) != 0) {
    printf("Error closing file %s\n",file);
    return(1);
  }
  pmat = matOpen(file, "r");
  if (pmat == NULL) {
    printf("Error reopening file %s\n", file);
    return(1);
  }

  /* Get headers of all variables */
  printf("\nExamining the header for each variable:\n");
  for (i=0; i < ndir; i++) {
    pa = matGetNextVariableInfo(pmat, &name);
    if (pa == NULL) {
	printf("Error reading in file %s\n", file);
	return(1);
    }
    /* Diagnose header pa */
    printf("According to its header, array %s has %d dimensions\n",
	   name, mxGetNumberOfDimensions(pa));
    if (mxIsFromGlobalWS(pa))
      printf("  and was a global variable when saved\n");
    else
      printf("  and was a local variable when saved\n");
    mxDestroyArray(pa);
  }

  /* Reopen file to read in actual arrays. */
  if (matClose(pmat) != 0) {
    printf("Error closing file %s\n",file);
    return(1);
  }
  pmat = matOpen(file, "r");
  if (pmat == NULL) {
    printf("Error reopening file %s\n", file);
    return(1);
  }

  /* Read in each array. */
  printf("\nReading in the actual array contents:\n");
  for (i=0; i<ndir; i++) {
      pa = matGetNextVariable(pmat, &name);
      if (pa == NULL) {
	  printf("Error reading in file %s\n", file);
	  return(1);
      } 
      /*
       * Diagnose array pa
       */
      printf("According to its contents, array %s has %d dimensions\n",
	     name, mxGetNumberOfDimensions(pa));
      if (mxIsFromGlobalWS(pa))
	printf("  and was a global variable when saved\n");
      else
	printf("  and was a local variable when saved\n");
      mxDestroyArray(pa);
  }

  if (matClose(pmat) != 0) {
      printf("Error closing file %s\n",file);
      return(1);
  }
  printf("Done\n");
  return(0);
}

int diag_main(int argc, char **argv)
{
  
  int result;

  if (argc > 1)
    result = diagnose(argv[1]);
  else{
    result = 0;
    printf("Usage: matdgns <matfile>");
    printf(" where <matfile> is the name of the MAT-file");
    printf(" to be diagnosed\n");
  }

  return (result==0)?EXIT_SUCCESS:EXIT_FAILURE;

}


