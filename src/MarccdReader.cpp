
#include <yat/threading/Mutex.h>
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include "Debug.h"
#include "Constants.h"
#include "Data.h"
#include "MarccdReader.h"
#include "MarccdInterface.h"


#define kLO_WATER_MARK      128
#define kHI_WATER_MARK      512

#define kPOST_MSG_TMO       2

const size_t kTASK_PERIODIC_TIMEOUT_MS	 =  1000;
const double kDEFAULT_READER_TIMEOUT_SEC = 60.;
const size_t MARCCD_START_MSG     =   (yat::FIRST_USER_MSG + 300);
const size_t MARCCD_RESET_MSG     =   (yat::FIRST_USER_MSG + 302);

//---------------------------
//- Ctor
//---------------------------
Reader::Reader(Camera& cam, HwBufferCtrlObj& buffer_ctrl)
      : _cam(cam),
        _buffer(buffer_ctrl),
	_image_number(0),
	_currentImgFileName(""),
	_simulated_image(0),
	_is_reader_open_image_file(true),  //- read image from file (no simulated image)
	_tmOut(0)
{
  DEB_CONSTRUCTOR();
}

//---------------------------
//- Dtor
//---------------------------
Reader::~Reader()
{
	DEB_DESTRUCTOR();
	if ( this->_tmOut )
	{
		delete _tmOut;
		_tmOut = 0;
	}
}

//---------------------------
//- Reader::start()
//---------------------------
void Reader::start()
{
	DEB_MEMBER_FUNCT();

	try
	{
		this->post(new yat::Message(MARCCD_START_MSG), kPOST_MSG_TMO);
	}
	catch (Exception &e)
	{
		// Error handling
		DEB_ERROR() << e.getErrMsg();
		throw LIMA_HW_EXC(Error, e.getErrMsg());
	}

}

//---------------------------
//- Reader::reset()
//---------------------------
void Reader::reset()
{
  DEB_MEMBER_FUNCT();
  try
    {  
      //- disable timeout
      this->_tmOut->disable();
      this->post(new yat::Message(MARCCD_RESET_MSG), kPOST_MSG_TMO);
    }
  catch (Exception &e)
    {
      // Error handling
      DEB_ERROR() << e.getErrMsg();
      throw LIMA_HW_EXC(Error, e.getErrMsg());
    }
  
}

//---------------------------
//- Reader::getLastAcquiredFrame()
//---------------------------
int Reader::getLastAcquiredFrame(void)
{
	yat::MutexLock scoped_lock(_lock);
	// _image_number corresponds to the image the reader looks for.
	return this->_image_number - 1;        
}

//---------------------------
//- Reader::isTimeoutSignaled()
//---------------------------
bool Reader::isTimeoutSignaled()
{
	bool expired = false;
	DEB_MEMBER_FUNCT();
	yat::MutexLock scoped_lock(_lock);
	if( this->_tmOut )
		expired = this->_tmOut->expired();
	else
		throw LIMA_HW_EXC(Error, "Reader initialisation failed : cannot get timeout state.");

	return expired;
}

//---------------------------
//- Reader::isRunning()
//---------------------------
bool Reader::isRunning(void)
{
	DEB_MEMBER_FUNCT();
	yat::MutexLock scoped_lock(_lock);
	return this->periodic_msg_enabled();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Reader::setTimeout(double newTimeOutVal)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(newTimeOutVal);
  yat::MutexLock scoped_lock(_lock);
  if ( this->_tmOut )
    this->_tmOut->set_value(newTimeOutVal);
  else
    throw LIMA_HW_EXC(Error, "Reader initialisation failed : cannot set new timeout value.");
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Reader::enableReader(void)
{
  DEB_MEMBER_FUNCT();
  yat::MutexLock scoped_lock(_lock);
  this->_is_reader_open_image_file = true;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Reader::disableReader(void)
{
  DEB_MEMBER_FUNCT();
  yat::MutexLock scoped_lock(_lock);
  this->_is_reader_open_image_file = false;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Reader::handle_message( yat::Message& msg )  throw( yat::Exception )
{
  DEB_MEMBER_FUNCT();
  //std::cout << "Reader yat message: " << msg.type() << std::endl;
  try
    {
      switch ( msg.type() )
	{
	  //-----------------------------------------------------    
	case yat::TASK_INIT:
	  {
	    DEB_TRACE() <<"Reader::->TASK_INIT";
	    //- create timeout
	    this->_tmOut = new yat::Timeout();
	    //- set unit in seconds
	    this->_tmOut->set_unit(yat::Timeout::TMO_UNIT_SEC);
	    //- set default timeout value
	    this->_tmOut->set_value(kDEFAULT_READER_TIMEOUT_SEC);
	  }
	  break;
	  //-----------------------------------------------------    
	case yat::TASK_EXIT:
	  {
	    DEB_TRACE() <<"Reader::->TASK_EXIT";
	  }
	  break;
	  //-----------------------------------------------------    
	case yat::TASK_TIMEOUT:
	  {
	    DEB_TRACE() <<"Reader::->TASK_TIMEOUT";
	  }
	  break;
	  //-----------------------------------------------------    
	case yat::TASK_PERIODIC:
	  {
	    DEB_TRACE() << "Reader::->TASK_PERIODIC";
	    //std::cout   << "Reader::->TASK_PERIODIC" << std::endl;
	    //- check if timeout expired
	    if ( this->_tmOut->expired() )
	      {
		DEB_TRACE() << "FATAL::Failed to load image : timeout expired !";
		//std::cout << "FATAL::Failed to load image : timeout expired !" 
		//	  << std::endl;
		
		//- disable periodic msg
		this->enable_periodic_msg(false);
		return;
	      }

	    //- get full image name as full/path/imgName_idx
	    std::stringstream newFileName;
	    newFileName << this->_cam.getImagePath()  
			<< this->_cam.getImageFileName()
			<< "_" <<  _image_number ;

	    // Force nfs file system to refresh
	    std::stringstream lsCommand;
	    lsCommand  << "ls " 
		       << this->_cam.getImagePath()
		       << "; ls "
		       << newFileName.str()
		       << " >& /dev/null" // avoid print out 
	      ;
	    system(lsCommand.str().c_str());

	    //- check if file exist	  
	    std::ifstream imgFile(newFileName.str().c_str());

	    if ( imgFile && this->_currentImgFileName != newFileName.str())
	      {
		this->_currentImgFileName = newFileName.str();
		//std::cout << "\t\t\t Reader: File [" << newFileName.str()
		//	  << "] exist ..." << std::endl;

		//- read image file
		this->getImageFromFile();
		
		int nb_frames;
		this->_cam.getNbFrames(nb_frames);

		// Wait for more frames? 
		if (++_image_number < 
		    this->_cam.getFirstImage() + nb_frames) 
		  {
		    this->_tmOut->restart();
		  }
		else 
		  {
		    //std::cout << "Reader: All images read." << std::endl;
		    //- disable periodic msg
		    this->_tmOut->disable();
		    this->enable_periodic_msg(false);
		  }
		
		//std::cout << "\t\t\t Reader::->TASK_PERIODIC ... DONE !!!" << std::endl;
	      }
	    else 
	      { 
		//std::cout << "\t\t\t Reader::->imgFile DOES NOT exist ..." << std::endl;
	      }
	  }
	  break;
	  //-----------------------------------------------------    
	case MARCCD_START_MSG:    
	  {
	    DEB_TRACE() << "Reader::->MARCCD_START_MSG";
	    //std::cout   << "Reader::->MARCCD_START_MSG" << std::endl;
	    //- enable periodic msg
	    this->enable_periodic_msg(true);
	    // Next image we are waiting for
	    this->_image_number = this->_cam.getFirstImage();
	    //- re-arm timeout
	    this->_tmOut->restart();
	    this->_currentImgFileName = string("");
	    //std::cout << "\t\t\t Reader::->MARCCD_START_MSG DONE for image name : " << this->_currentImgFileName << std::endl;
	  }
	  break;
	  //-----------------------------------------------------
	case MARCCD_RESET_MSG:
	  {
	    DEB_TRACE() << "Reader::->MARCCD_RESET_MSG";
	    //std::cout   << "Reader::->MARCCD_RESET_MSG" << std::endl;
	    this->enable_periodic_msg(false);
	    this->_image_number = this->_cam.getImageIndex();
	    // Clean buffer
	    std::stringstream rmCommand;
	    rmCommand  << "rm " << this->_cam.getImagePath()  
		       << this->_cam.getImageFileName()
		       << "_* >& /dev/null" ; // avoid print out
	    system(rmCommand.str().c_str());
	  }
	  break;    
	  //-----------------------------------------------------
	}
    }
  catch( yat::Exception& ex )
    {
      DEB_ERROR() <<"Error : " << ex.errors[0].desc;
      throw;
    }
}
//-----------------------------------------------------
bool Reader::getImageFromFile ()
{  

  StdBufferCbMgr& buffer_mgr = ((reinterpret_cast<BufferCtrlObj&>(this->_buffer)).getBufferCbMgr());

  // Buffer manager is very strict with the data pointer, 
  // it will accept only its own pointer address
  int frameNumber = this->_image_number - this->_cam.getFirstImage();
  void *ptr = buffer_mgr.getFrameBufferPtr(frameNumber);

  buffer_mgr.setStartTimestamp(Timestamp::now());

  HwFrameInfoType frame_info;
  frame_info.acq_frame_nb = frameNumber ;

  // Read data buffer from file
  char filename[this->_currentImgFileName.size() + 1];
  strcpy(filename,this->_currentImgFileName.c_str());
  DI::DiffractionImage *tmpDI = new DI::DiffractionImage(filename);
  
  //Check that the image size corresponds with the expected frame size
  Size frameSize = buffer_mgr.getFrameDim(). getSize();
  if( frameSize.getWidth()  != tmpDI->getWidth() ||
      frameSize.getHeight() != tmpDI->getHeight())
    throw LIMA_HW_EXC(Error, "Image size in file is different from the expected image size of this detector !");
  
  // Copy data buffer without modifying the pointer
  int img_size = tmpDI->getWidth() * tmpDI->getHeight();
  unsigned int *image = tmpDI->getImage();	
  for(int j=0;j<img_size;j++)
    {
      ((uint16_t*)ptr)[j] = image[j];
    }

  delete tmpDI;

  //std::cout << "\t\t\tnew frame ready" << std::endl; 
  return buffer_mgr.newFrameReady(frame_info);  
}

