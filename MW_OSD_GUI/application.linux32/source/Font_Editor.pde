PImage FullFont;

public boolean FontChanged = false;
public boolean mouseDown = false;
public boolean mouseUp = true;
PImage PreviewChar;
PImage[] CharImages = new PImage[256];
int row = 0;
int gap = 5;
int gapE = 1;
int curPixel = -1;
int curChar = -1;
int editChar = -1;
boolean mouseSet = false;
color gray = color(120);
color white = color(255);
color black = color(0);


// screen locations
int XFullFont = 25;    int YFullFont = 25;
int XcharEdit = 5;    int YcharEdit = 5;
int PreviewX = 60;     int PreviewY = 275;



Bang CharBang[] = new Bang[256] ;
Bang CharPixelsBang[] = new Bang[216] ;
Bang PreviewCharBang;
Textlabel CharTopLabel[] = new Textlabel[16] ;
Textlabel CharSideLabel[] = new Textlabel[16] ;
Textlabel LabelCurChar;
Group FG,FGFull,FGCharEdit, FGPreview;
Button buttonSendFile,buttonBrowseFile,buttonEditFont,buttonFClose,buttonFSave;


void Font_Editor_setup() {
  for (int i=0; i<256; i++) {
    CharImages[i] = createImage(12, 18, ARGB);
  }
  
 
  
  FG = FontGroupcontrolP5.addGroup("FG")
    .setPosition(150,50)
    .setWidth(680)
    .setBarHeight(12)
    .activateEvent(true)
    .setBackgroundColor(color(200,255))
    .setBackgroundHeight(450)
    .setLabel("Font Editor")
    .setMoveable(true)
    .disableCollapse()
    .hide();
    
    ;
    FGFull = FontGroupcontrolP5.addGroup("FGFull")
    .setPosition(20,20)
    .setWidth( 16*(12+gap)+ 25)
    .setBarHeight(12)
    .activateEvent(true)
    .hideBar() 
    .setBackgroundColor(color(0,255))
    .setBackgroundHeight(16*(18+gap)+35)
    .setLabel("Character List")
    .setMoveable(true)
    .setGroup(FG);
    ;
    FGCharEdit = FontGroupcontrolP5.addGroup("FGCharEdit")
    .setPosition(350,20)
    .setWidth( 12*(10+gapE)+ 33)
    .setBarHeight(12)
    .activateEvent(true)
    .hideBar() 
    .setBackgroundColor(color(180,255))
    .setBackgroundHeight(18*(10+gapE)+120)
    .setMoveable(true)
    .setGroup(FG);
    ;
    
    
    
// RawFont = LoadFont("MW_OSD.mcm");
for (int i=0; i<256; i++) {
    int boxX = XFullFont+(i % 16) * (12+gap);
    int boxY = YFullFont + (i / 16) * (18+gap);
    CharBang[i] = FontGroupcontrolP5.addBang("CharMap"+i)
      .setPosition(boxX, boxY)
      .setSize(12, 18)
      .setLabel("")
      .setId(i)
      .setImages(CharImages[i], CharImages[i], CharImages[i], CharImages[i]) 
      .setGroup(FGFull);
    ;
   
  } 
String[] CharRows = {"0","1","2","3","4","5","6","7","8","9","A","B","C","D","E","F"};
  
  for (int i=0; i<16; i++) {
    int boxX = XFullFont-15;
    int boxY = YFullFont + i * (18+gap);
    CharSideLabel[i] = FontGroupcontrolP5.addTextlabel("charlabside" +i,CharRows[i],boxX ,boxY )
    .setGroup(FGFull)
    .setColor(white);
    ;
  } 
  for (int i=0; i<16; i++) {
    int boxX = XFullFont + i * (12+gap);
    int boxY = YFullFont-15;
    CharTopLabel[i] =FontGroupcontrolP5.addTextlabel("charlabtop" +i,CharRows[i],boxX ,boxY )
    .setGroup(FGFull)
    .setColor(white);
    ;
  } 
  

 
 for (int i=0; i<216; i++) {
    int boxX = XcharEdit+(i % 12) * (12+gapE);
    int boxY = YcharEdit + (i / 12) * (12+gapE);
    CharPixelsBang[i] = FontGroupcontrolP5.addBang("CharPix"+i)
      .setPosition(boxX, boxY)
      .setSize(12, 12)
      .setLabel("")
      .setId(i)
      .setColorBackground(gray)
      .setColorForeground(gray)
      .setValue(0)
      .setGroup(FGCharEdit)
      ;
    
  }

  LabelCurChar = FontGroupcontrolP5.addTextlabel("LabelCurChar","No Index Set" ,XcharEdit+ 10,YcharEdit + 18*(12+gapE)+5)
    .setColor(white)
    .setGroup(FGCharEdit);
 
  PreviewCharBang = FontGroupcontrolP5.addBang("PreviewCharBang")
    .setPosition(FGCharEdit.getWidth() / 2 -6, PreviewY)
    .setSize(12, 18)
    .setLabel("")
    //.setId(i)
    .setImages(PreviewChar, PreviewChar, PreviewChar, PreviewChar) 
    .setGroup(FGCharEdit);
  ;
  buttonFSave = FontGroupcontrolP5.addButton("FSave",1,FGCharEdit.getWidth()-55, FGCharEdit.getBackgroundHeight()-25,45,18);//buttonFClose.setColorBackground(red_);
  buttonFSave.getCaptionLabel()
    .setFont(font12)
    .toUpperCase(false)
    .setText("SAVE");
  buttonFSave.setGroup(FGCharEdit); 
 
 
 
  buttonFClose = FontGroupcontrolP5.addButton("FCLOSE",1,680- 55 ,10,45,18);buttonFClose.setColorBackground(red_);
  buttonFClose.getCaptionLabel()
    .setFont(font12)
    .toUpperCase(false)
    .setText("CLOSE");
  buttonFClose.setGroup(FG);
 
  MGUploadF = controlP5.addGroup("MGUploadF")
                .setPosition(XFONTTOOLS,YFONTTOOLS)
                .setWidth(110)
                .setBarHeight(15)
                .activateEvent(true)
                .setBackgroundColor(color(30,255))
                .setBackgroundHeight(70)
                .setLabel("     Font Tools")
                .disableCollapse();
                //.close() 
               ; 

FileUploadText = controlP5.addTextlabel("FileUploadText","",10,5)
.setGroup(MGUploadF)
.setColorBackground(font_);

buttonEditFont = controlP5.addButton("EditFont",1,20,45,60,16)
.setColorBackground(font_)
.setGroup(MGUploadF);
buttonEditFont.getCaptionLabel()
.setText("Edit Font");

buttonSendFile = controlP5.addButton("FONT_UPLOAD",1,20,25,60,16)
.setColorBackground(font_)
.setGroup(MGUploadF);
buttonSendFile.getCaptionLabel()
.setText("UPLOAD");

buttonBrowseFile = controlP5.addButton("Browse",1,20,5,60,16)
.setColorBackground(font_)
.setGroup(MGUploadF);
buttonBrowseFile.getCaptionLabel()
.setText("  SELECT")
;

buttonSendFile.getCaptionLabel()
    .toUpperCase(false)
    .setText(" UPLOAD");
 
}

public void Send(){
  //sendFontFile();
  //g_serial.clear();
  //CreateFontFile();
}

void MakePreviewChar(){
  PImage PreviewChar = createImage(12, 18, ARGB);
   PreviewChar.loadPixels();
  for(int byteNo = 0; byteNo < 216; byteNo++) {
    switch(int(CharPixelsBang[byteNo].value())) {
      case 0:
        PreviewChar.pixels[byteNo] = gray;
        //CharImages[charNo].pixels[CharIndex] = gray;
      break;     
      case 1:
        PreviewChar.pixels[byteNo] = black;
        //CharImages[charNo].pixels[CharIndex] = black;
      break; 
      case 2:
        PreviewChar.pixels[byteNo] = white;
        //CharImages[charNo].pixels[CharIndex] = white;
      break; 
    }
  }
  PreviewChar.updatePixels();
  PreviewCharBang.setImages(PreviewChar, PreviewChar, PreviewChar, PreviewChar); 
}

void FSave(){
  UpdateChar();
  CreateFontFile();
}

void FCLOSE(){
  Lock_All_Controls(false);
  FG.hide();
}

void EditFont(){
  Lock_All_Controls(true);
 //setLock(ScontrolP5.getController("MwHeading"),true);
  FG.show();
  Lock_All_Controls(true);
}

void changePixel(int id){
 
  if ((mouseUp) && (id > -1)){
    int curColor = int(CharPixelsBang[id].value());
    switch(curColor) {
    
    case 0: 
     CharPixelsBang[id].setColorForeground(black);
     CharPixelsBang[id].setValue(1);
     FontChanged = true;
     //println("0");
    
    break; 
    
    case 1: 
     CharPixelsBang[id].setColorForeground(white);
     CharPixelsBang[id].setValue(2);
     FontChanged = true;
     //println("1");
    
    break; 
    
    case 2: 
     CharPixelsBang[id].setColorForeground(gray);
     CharPixelsBang[id].setValue(0);
     FontChanged = true;
     //println("2");
    
    break; 
   }
  }
  curPixel = -1;
  MakePreviewChar();
 
}  

void GetChar(int id){
 
  if ((mouseUp) && (id > -1)){
    for(int byteNo = 0; byteNo < 216; byteNo++) {
      CharPixelsBang[byteNo].setColorForeground(gray);
      CharPixelsBang[byteNo].setValue(0);
    }
    for(int byteNo = 0; byteNo < 216; byteNo++) {
      switch(CharImages[id].pixels[byteNo]) {
          case 0xFF000000:
            CharPixelsBang[byteNo].setColorForeground(black);
            CharPixelsBang[byteNo].setValue(1);
          break; 
          case 0xFFFFFFFF:
            CharPixelsBang[byteNo].setColorForeground(white);
            CharPixelsBang[byteNo].setValue(2);
          break;
          default:
            CharPixelsBang[byteNo].setColorForeground(gray);
            CharPixelsBang[byteNo].setValue(0);
          break;  
      }
      
    }
   
    LabelCurChar.setValue(str(id));
    LabelCurChar.setColorBackground(0);
    MakePreviewChar();
    editChar = id;
    //LabelCurChar.update(); 
    curChar = -1;
  }
 
}


void UpdateChar(){
 int changechar = Integer.parseInt(LabelCurChar.getStringValue());

 
  CharImages[changechar].loadPixels();
  for(int byteNo = 0; byteNo < 216; byteNo++) {
    switch(int(CharPixelsBang[byteNo].value())) {
      case 0:
       CharImages[changechar].pixels[byteNo] = gray;
        //CharImages[charNo].pixels[CharIndex] = gray;
      break;     
      case 1:
       CharImages[changechar].pixels[byteNo] = black;
        //CharImages[charNo].pixels[CharIndex] = black;
      break; 
      case 2:
       CharImages[changechar].pixels[byteNo] = white;
        //CharImages[charNo].pixels[CharIndex] = white;
      break; 
    }
  }
  CharImages[changechar].updatePixels();
  CharBang[changechar].setImages(CharImages[changechar], CharImages[changechar], CharImages[changechar], CharImages[changechar]); 
}    
  
  
  




void setLock(Controller theController, boolean theValue) {
  theController.setLock(theValue);
}

void Lock_All_Controls(boolean theLock){

 //System.out.println(controlP5.getControllerList());
ControllerInterface[] sctrl = ScontrolP5.getControllerList();
   for(int i=0; i<sctrl.length; ++i)
   {
    try{
      setLock(ScontrolP5.getController(sctrl[i].getName()),theLock);
    }catch (NullPointerException e){}
   }
   
   ControllerInterface[] ctrl5 = controlP5.getControllerList();
   for(int i=0; i<ctrl5.length; ++i)
   {
    try{
      setLock(controlP5.getController(ctrl5[i].getName()),theLock);
    }catch (NullPointerException e){}
   }
}

public void Browse(){
  SwingUtilities.invokeLater(new Runnable(){
    public void run(){
      final JFileChooser fc = new JFileChooser(dataPath(""));
      fc.setDialogType(JFileChooser.SAVE_DIALOG);
      fc.setFileFilter(new FontFileFilter());
      //fc.setCurrentDirectory();
      int returnVal = fc.showOpenDialog(null);
      if (returnVal == JFileChooser.APPROVE_OPTION) {
        File FontFile = fc.getSelectedFile();
        FileInputStream in = null;
        boolean completed = false;
        String error = null;
        try{
          in = new FileInputStream(FontFile) ;
          FontFileName = FontFile.getPath();
          img_Clear = LoadFont(FontFileName);
          updateConfig(); 
          JOptionPane.showMessageDialog(null,new StringBuffer().append("Font File loaded : ").append(FontFile.toURI()) );
          completed  = true;
          
        }catch(FileNotFoundException e){
                error = e.getCause().toString();

        }catch( IOException ioe){/*failed to read the file*/
                ioe.printStackTrace();
                error = ioe.getCause().toString();
        }finally{
          if (!completed){
                 // MWI.conf.clear();
                 // or we can set the properties with view values, sort of 'nothing happens'
                 //updateModel();
          }
          //updateView();
          if (in!=null){
            try{
              in.close();
            }catch( IOException ioe){/*failed to close the file*/}
          }
          
          if (error !=null){
                  JOptionPane.showMessageDialog(null, new StringBuffer().append("error : ").append(error) );
          }
        }
      }
    }
  }
  );  
}


public class FontFileFilter extends FileFilter {
  public boolean accept(File f) {
    if(f != null) {
      if(f.isDirectory()) {
        return true;
      }
      String extension = getExtension(f);
      if("mcm".equals(extension)) {
        return true;
      }
    }
    return false;
  }

  public String getExtension(File f) {
    if(f != null) {
      String filename = f.getName();
      int i = filename.lastIndexOf('.');
      if(i>0 && i<filename.length()-1) {
        return filename.substring(i+1).toLowerCase();
      }
    }
    return null;
  } 

  public String getDescription() {
    return "*.mcm Font File";
  }   
}
