byte[][] raw_font;
PrintWriter  Output;

PImage LoadFont(String filename) {
  //System.out.println("LoadFont "+filename);
  raw_font = LoadRawFont(filename);
  return RawFontToImage(raw_font);
}


byte[][] LoadRawFont(String filename) {
  byte[][] raw = new byte[256][54];

  InputStream in = null;
  
  byte[] header = { 'M','A','X','7','4','5','6' };
  boolean inHeader = true;
  int hIndex = 0;
  int bitNo = 0;
  int byteNo = 0;
  int charNo = 0;
  int curByte = 0;
 
  try {
    in = createInput(filename);
    
    while(in.available() > 0) {
      int inB = in.read();
     
      if(inHeader) {
        if(hIndex < header.length && header[hIndex] == inB) {
          hIndex++;
          continue;
        }
        if(hIndex == header.length && (inB == '\r' || inB == '\n')) {
          inHeader = false;
          //System.out.println("done header");
          continue;
        }
        hIndex = 0;
        continue;
      }
      else {
        switch(inB) {
        case '\r':
        case '\n':
          if (bitNo == 0)
            continue; 
          if (bitNo == 8) {
            if (byteNo < 54) {
              //System.out.println("*charNo="+charNo+" byteNo="+byteNo);//+" value="+(int)raw[charNo][byteNo]);
              raw[charNo][byteNo] = (byte)curByte;
            }
            bitNo = 0;
          
            curByte = 0;
            ++byteNo;
            if(byteNo == 64) {
              //System.out.println("Loaded char "+charNo);
              byteNo = 0;
              ++charNo;
            }
          }
          break;
        case '0':
        case '1':
          if(bitNo >= 8) {
            throw new Exception("File format error");
          }
          curByte = (curByte << 1) | (inB & 1);
          ++bitNo;
          break;
        }
      }
    }
  }
  catch (FileNotFoundException e) {
      System.out.println("File Not Found "+filename);
  }
  catch (IOException e) {
      System.out.println("IOException");
  }
  catch(Exception e) {
      System.out.println("Exception");
  }
  finally {
    if(in != null)
      try {
        in.close();
      }
      catch (IOException ioe) {
      }
  }
  
  return raw;
}

PImage RawFontToImage(byte[][] raw) {
  PImage img = createImage(12, 18*256, ARGB);
  img.loadPixels();

  // Pixel values
  int white = 0xFFFFFFFF;
  int black = 0xFF000000;
  int transparent = 0x00000000;
  color gray = color(120);

  for(int charNo = 0; charNo < 256; charNo++) {
    CharImages[charNo].loadPixels();
   // CharImages[charNo] = createImage(12, 18, ARGB);
    for(int byteNo = 0; byteNo < 54; byteNo++) {
      for(int i = 0; i < 4; i++) {
	int index = (charNo*12*18) + (byteNo*4) + (3-i);
        int CharIndex = (byteNo*4) + (3-i);
        //System.out.println("charNo="+charNo+" byteNo="+byteNo+" index="+index+" CharIndex="+CharIndex+" Charlimit="+CharImages[charNo].pixels.length + " limit="+img.pixels.length); 
        int curByte = (int)raw[charNo][byteNo];
	switch((curByte >> (2*i)) & 0x03) {
	case 0x00:
	   img.pixels[index] = black;
            CharImages[charNo].pixels[CharIndex] = black;
	   break; 
	case 0x01:
	   img.pixels[index] = transparent;
            CharImages[charNo].pixels[CharIndex] = gray;
	   break; 
	case 0x02:
	   img.pixels[index] = white;
            CharImages[charNo].pixels[CharIndex] = white;
	   break; 
	case 0x03:
	   img.pixels[index] = transparent;
            //CharImages[charNo].pixels[CharIndex] = gray;

	   break; 
	}
      }
    }
    CharImages[charNo].updatePixels();
  }

  img.updatePixels();
  return img;
}

void CreateFontFile(){
  String gray = "01";  // gray in Gui transparrent in OSD
  String black = "00"; //black
  String white = "10";
  int PixelCounter = 0;
  int fullpixels = 0;
  String OutputLine = "";
  
  Output = createWriter("data/custom.mcm");
  
  Output.println("MAX7456"); // write header
  for(int id = 0; id < 256; id++) {
    for(int byteNo = 0; byteNo < 216; byteNo++) {
      switch(CharImages[id].pixels[byteNo]) {
        case 0xFF000000:
          OutputLine+=black;
          PixelCounter+=1;
        break; 
        case 0xFFFFFFFF:
          OutputLine+=white;
          PixelCounter+=1;
        break;
       default:
         OutputLine+=gray;
         PixelCounter+=1;
        break;  
      }
      
    if(PixelCounter == 4){
      Output.println(OutputLine);
      OutputLine = "";
      PixelCounter = 0;
    }
     
    }
    for(int spacer = 0; spacer < 10; spacer++) {
      Output.println("01010101");
    }
  }
  
 //Output.println("done");
 Output.flush(); // Writes the remaining data to the file
  Output.close();  
  img_Clear = LoadFont("/data/default.mcm");  
}


  
