/// place_rec_bits.c (c) Adrien Angeli, Imperial College London, 2009


/// Number of bins (measurements) considered to characterize one location
#define NO_BINS 360

/// Number of locations we want to learn in the environment
#define NO_LOCS 5

/// The size of the data that is going to be written to files
int nFileSize	= NO_BINS * 2;

/// A table containing the names of the files we are going to read / write
string file_names[NO_LOCS];


/// The location signature structure: stores the signature characterizing one location
typedef struct
{
	short sig[NO_BINS];
} loc_sig;



// --------------------- File management functions ---------------
/** This function fills the file_names global variable with names
    like loc_%%.dat where %% are 2 digits (00, 01, 02...) indicating
    the location number. Up to NO_LOCS (i.e, the number of locations
    in the environment) are going to be assigned a file name */
void form_file_names()
{
  for ( short n=0; n<NO_LOCS; n++ )
    {
      StringFormat(file_names[n], "loc_%02d.dat", n);
    }
}



/** Given that some location files have been stored, and given
    that at most NO_LOCS are going to be learned, what is the
    next location index that is expected to be written? For instance,
    if 2 locations out of 3 have already been recorded in files
    loc_00.dat and loc_01.dat, then, the next "free" index for
    naming a file would be 2, and the corresponding file would be
    loc_02.dat */
short find_next_loc_index()
{
  short n = 0;
  while ( n<NO_LOCS )
    {
      TFileIOResult nIoResult;
      TFileHandle hFileHandle;
      int read_size;
      
      OpenRead(hFileHandle, nIoResult, file_names[n], read_size);
      bool exists = (nIoResult == 0);
      Close(hFileHandle, nIoResult);
      if ( !exists )
	{
	  Delete(file_names[n], nIoResult);
	  break;
	}
      else
	n++;
    }
  
  return n;
}


/// Delete all loc_%%.dat files
void delete_loc_files()
{
  for ( short n=0; n<NO_LOCS; n++ )
    {
      TFileIOResult nIoResult;
      TFileHandle hFileHandle;
      int read_size;
      
      OpenRead(hFileHandle, nIoResult, file_names[n], read_size);
      bool exists = (nIoResult == 0);
      Close(hFileHandle, nIoResult);
      if ( exists )
	Delete(file_names[n], nIoResult);
    }
}



/** Writes the signature ls to the file whose named can be
    identified using index (e.g, if index is 1, then ls is going
    to be written in file loc_01.dat). If file already exists,
    it should be replaced (FOR SECURITY, IT IS BETTER THAT FILE
    DID NOT ALREADY EXIST). */
void write_signature_to_file(loc_sig& ls, short index)
{
  ASSERT(index >= 0 && index < NO_LOCS);
  TFileIOResult nIoResult;
  TFileHandle hFileHandle;

  string sFileName = file_names[index];
  Delete(sFileName, nIoResult);
  
  OpenWrite(hFileHandle, nIoResult, sFileName, nFileSize);
  for ( short i=0; i<NO_BINS; ++i )
    WriteShort(hFileHandle, nIoResult, ls.sig[i]);
  
  Close(hFileHandle, nIoResult);
}



/** Read file corresponding to index (e.g., if index is 1,
    read file will be loc_01.dat), and store its signature
    into ls. ASSUMES FILE EXISTS! */
void read_signature_from_file(loc_sig& ls, short index)
{
  ASSERT(index >= 0 && index < NO_LOCS);
  TFileIOResult nIoResult;
  TFileHandle hFileHandle;
  int read_size;

  string sFileName = file_names[index];
  OpenRead(hFileHandle, nIoResult, sFileName, read_size);
  for ( short i=0; i<NO_BINS; ++i )
    ReadShort(hFileHandle, nIoResult, ls.sig[i]);
  
  Close(hFileHandle, nIoResult);
}




/** This function simply characterizes the current location,
    and stores the obtained signature into the next available
    file index (e.g., 2 if loc_00.dat and loc_01.dat already
    exist). WARNING: AT MOST NO_LOCS CAN BE LEARNED. */
void learn_location()
{
	loc_sig ls;
	// characterize_location(ls);
	short index = find_next_loc_index();
	write_signature_to_file(ls, index);
}



/** This function tries to recognize the current location.
    1. Characterize current location
    2. For every learned locations
    2.1. Read signature of learned location from file
    2.2. Compare signature to signature coming from actual characterization
    3. Retain the learned location whose minimum distance with
    actual characterization is the smallest.
    4. Display the index of the recognized location on the screen */
void recognize_location()
{
  loc_sig ls_obs;
  //characterize_location(ls_obs);
  
  for ( short n=0; n<NO_LOCS; n++ )
    {
      loc_sig ls_read;
      read_signature_from_file(ls_read, n);
      
      // FILL IN: COMPARE ls_read with ls_obs and find the best match
    }
  
  // Display output
}



/** First thing to do, is to form file names.
    Prior to starting learning the locations, it
    could be good to delete files from previous
    learning by calling delete_loc_files(). This
    could also be done manually using RobotC. Then,
    either learn a location, until all the locations
    are learned, or try to recognize one of them, if
    locations have already been learned. Last infinite
    while loop is just for the display to hold on. */
task main()
{
  form_file_names();
  //delete_loc_files();
  
  //learn_location();
  
  //recognize_location();

  while ( true )
  {
    wait10Msec(100);
  }

  return;
}
