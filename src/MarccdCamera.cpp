#include "MarccdCamera.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>
#include <time.h>
#include <yat/utils/XString.h>


using namespace lima;
using namespace lima::Marccd;
//using namespace std;

const double PixelSize = 39.795;

//- User task messages
const size_t  START_MSG			=	(yat::FIRST_USER_MSG + 100);
const size_t  STOP_MSG			=	(yat::FIRST_USER_MSG + 101);
const size_t  GET_IMAGE_MSG		=	(yat::FIRST_USER_MSG + 200);
const size_t  BACKGROUND_FRAME_MSG	=	(yat::FIRST_USER_MSG + 300);

//- task period (in ms)
const size_t kPERIODIC_MSG_PERIOD = 500;

const size_t DATA_SIZE   = 16;			//- 16 bits image
const size_t DATA_IMAGE_OFFSET = 4096;	//- image data offset in the Marccd file
//- Task numbers
const size_t TASK_ACQUIRE  =   0;
const size_t TASK_READ     =   1;
const size_t TASK_CORRECT  =   2;
const size_t TASK_WRITE    =   3;
const size_t TASK_DEZINGER =   4;

//- The status bits for each task are:
//- Task Status bits
const size_t TASK_STATUS_QUEUED    = 0x1;
const size_t TASK_STATUS_EXECUTING = 0x2;
const size_t TASK_STATUS_ERROR     = 0x4;
const size_t TASK_STATUS_RESERVED  = 0x8;

//- These are the "old" states from version 0, but BUSY is also used in version 1
const size_t TASK_STATE_IDLE        = 0;
const size_t TASK_STATE_ACQUIRE     = 1;
const size_t TASK_STATE_READOUT     = 2;
const size_t TASK_STATE_CORRECT     = 3;
const size_t TASK_STATE_WRITING     = 4;
const size_t TASK_STATE_ABORTING    = 5;
const size_t TASK_STATE_UNAVAILABLE = 6;
const size_t TASK_STATE_ERROR       = 7;	//- command not understood
const size_t TASK_STATE_BUSY        = 8;	//- interpreting command

//- These are the definitions of masks for looking at task state bits
#define STATE_MASK        0xf
#define STATUS_MASK       0xf
#define TASK_STATUS_MASK(task) (STATUS_MASK << (4*((task)+1)))

//- These are some convenient macros for checking and setting the state of each task
//- They are used in the marccd code and can be used in the client code
#define TASK_STATE(current_status) ((current_status) & STATE_MASK)
#define TASK_STATUS(current_status, task) (((current_status) & TASK_STATUS_MASK(task)) >> (4*((task) + 1)))
#define TEST_TASK_STATUS(current_status, task, status) (TASK_STATUS(current_status, task) & (status))

//- Frame type choices
enum {
    marccdFrameNormal,
    marccdFrameBackground,
    marccdFrameRaw,
    marccdFrameDoubleCorrelation
};

//---------------------------
//- Ctor
//---------------------------
Camera::Camera(const std::string& camera_ip, 
	       size_t port_num, 
	       const std::string& img_path): 
  _sock(0),
  _image_path(img_path),
  _image_name("buffer"),
  _full_img_name(""),
  m_nb_frames(0),
  m_image_size(0),
  m_trigger_type(0),
  m_exp_time(0),
  m_lat_time(0),
  m_binning(2),
  _marccd_state(TASK_STATE_IDLE),
  m_status(Camera::Unknown),
  _image_number(0),
  _first_image(0),
  _camera_ip(camera_ip),
  _port_num(port_num),
  _detector_model(""),
  _detector_type(""),
  _stop_sequence_finished(false),
  _abort(false),
  _bgAcquired(false),
  _saveBG(false),
  _error("")
{
  DEB_CONSTRUCTOR();
  
  //std::cout   << "Camera::Camera() - ENTERING" << std::endl;
  DEB_TRACE() << "Camera::Camera() - ENTERING ...";
  
  m_status = Camera::Ready;
  
  _detector_type  = "MARCCD" ;
  _detector_model = "SX 165";
  
  //std::cout   << "Camera::Camera() - DONE" << std::endl;
  DEB_TRACE() << "Camera::Camera() - DONE";

}

//---------------------------
//- Copy Ctor
//---------------------------

Camera::Camera(const Camera &other_cam) 
{
  *this = other_cam;
}

//---------------------------
//- Dtor
//---------------------------
Camera::~Camera()
{
  DEB_DESTRUCTOR();
  DEB_TRACE() << "Camera::~Camera() - ENTERING ...";
  //std::cout <<"Camera::~Camera() - ENTERING ..." << std::endl;
  
  DEB_TRACE() << "Camera::~Camera() - MARCCD DONE";
  //std::cout <<"Camera::~Camera() - MARCCD DONE" << std::endl;
}

//---------------------------
//- Operator =
//---------------------------
Camera& Camera::operator=(const Camera& other_cam)
{
  
  m_nb_frames 			= other_cam.m_nb_frames;
  m_image_size 			= other_cam.m_image_size;
  m_trigger_type 		= other_cam.m_trigger_type;
  m_exp_time 			= other_cam.m_exp_time;  
  m_lat_time 			= other_cam.m_lat_time;  
  m_binning 			= other_cam.m_binning;  
  m_status                      = other_cam.m_status;
  
  _marccd_state                 = other_cam._marccd_state;
  _stop_sequence_finished       = other_cam._stop_sequence_finished;
  _abort                        = other_cam._abort;
  
  _image_number                 = other_cam._image_number;
  _first_image                  = other_cam._first_image;
  _image_path                   = other_cam._image_path;
  _image_name                   = other_cam._image_name;
  _full_img_name                = other_cam._full_img_name;
  
  _camera_ip                    = other_cam._camera_ip;
  _port_num			= other_cam._port_num;
  _detector_model		= other_cam._detector_model;
  _detector_type     		= other_cam._detector_type;
  
  return *this;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::handle_message( yat::Message& msg )  throw( yat::Exception )
{
  DEB_MEMBER_FUNCT();
  //std::cout << "Camera yat message: " << msg.type() << std::endl;
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC,&now);
  static size_t lastmsg = 0;

  if (msg.type() != lastmsg)
    {
      std::cout << "Camera yat message: " << msg.type() << " (t " 
	//<< (now.tv_sec + (double) now.tv_nsec/1000000000 )
		<< now.tv_sec << "." << now.tv_nsec
		<< ")\n";
      lastmsg = msg.type();
    }
  try
    {
      switch ( msg.type() )
	{
	  //-----------------------------------------------------
	case yat::TASK_INIT:
	  {
	    DEB_TRACE() << "Camera::->TASK_INIT";
	    
	    try
	      {
		//- internal YAT stuff cooking
		yat::Socket::init();
		//- create socket
		_sock = new yat::ClientSocket ();
		//- connect to MARCCD device
		this->connect();
		//- no error
		this->_error.clear();
	      }
	    catch (yat::Exception &ye)
	      {
		this->enable_periodic_msg(false);
		//- update error
		this->_error = ye.errors[0].desc;
		//this->status_str = "Camera::Camera : device initialization failed caught DevFailed trying to create yat::Socket\n";
		if ( _sock )
		  delete _sock;
		_sock = 0;
		//std::cout << "Camera::Camera : Camera::handle_message initialization failed caught yat::Exception trying to create yat::ClientSocket" 
		//<< "\n\t_sock = " << _sock
		//<< std::endl;
		//std::cout << " caught YAT Exception [" << ye.errors[0].desc << "]" << std::endl;
		return;
	      }
	    catch (...)
	      {
		this->enable_periodic_msg(false);
		//std::cout << "Camera::handle_message : Camera initialization failed caught ... trying to create yat::ClientSocket" << std::endl;
		//this->status_str = "Camera::init_device : device initialization failed caught ... trying to create yat::Socket\n";
		if ( _sock )
		  delete _sock;
		_sock = 0;
		return;
	      }
	    
	    if ( this->_sock )
	      {
		this->enable_periodic_msg(true);
		this->set_periodic_msg_period( kPERIODIC_MSG_PERIOD );

		//- get image size to initialize ROI
		Size size;
		this->getImageSize(size);
		std::cout << "Set the ROI to full frame" << std::endl;
		Roi aFullFrame(0,0,size.getWidth(),size.getHeight());
		this->setRoi(aFullFrame);
	      }
	    else
	      {
		DEB_TRACE() << "TASK INIT -> CONNECTION FAILED !!!";
	      //std::cout << " TASK INIT -> CONNECTION FAILED !!!" << std::endl;
	      }
	    //std::cout << "Camera::->TASK_INIT DONE." << std::endl;
	    DEB_TRACE() << "Camera::->TASK_INIT DONE.";
		
	  }
	  break;
	  //-----------------------------------------------------
	case yat::TASK_EXIT:
	  {
	    DEB_TRACE() << "Camera::->TASK_EXIT";
	    //- check if connection is up
	    if( !this->_sock )
	      {
		//		status = Camera::Fault;
		//std::cout << "Camera::handle_message -> no _sock !" << std::endl;
		DEB_ERROR() << "Camera::handle_message -> no _sock !";
		return;
	      }
	    try
	      {
		//- abort acquisition
		this->write_read("abort");
		
		//-  Delete device allocated objects
		this->disconnect ();
		//std::cout << "Camera::handle_message -> disconnect ... " << std::endl;
		
		//- yat internal cooking
		yat::Socket::terminate();
		//std::cout << "Camera::handle_message -> terminate ... " << std::endl;
	      }
	    //- TODO : catch all exceptions !
	    catch (...)
	      {
		// Error handling
		std::cerr << "MARCCD TASK_EXIT -> An ... exception occurred!"  << std::endl;
	      }
	    DEB_TRACE() << "Camera::->TASK_EXIT DONE.";
	    //std::cout << "Camera::->TASK_EXIT DONE." << std::endl;
	  }
	  break;
	  //-----------------------------------------------------
	case yat::TASK_TIMEOUT:
	  {
	    DEB_TRACE() << "Camera::->TASK_TIMEOUT";
	    
	    //std::cout << "Camera::->TASK_TIMEOUT DONE." << std::endl;
	  }
	  break;
	  //- TASK_PERIODIC ===================
	case yat::TASK_PERIODIC:
	  {
	    DEB_TRACE() << "Camera::->TASK_PERIODIC";
	    //- code relative to the task's periodic job goes here
	    this->get_marccd_state();
	  }
	  break;
	  //- USER_DEFINED_MSG ================
	case START_MSG:
	  {
	    DEB_TRACE() << "Camera::->START_MSG";
	    
	    try 
	      {
		this->perform_acquisition_sequence();
	      }
	    catch (Exception e)
	      {
		m_nb_frames = 0; // This will stop the reader
		//std::cout << "ERROR: Camera::->START_MSG --> " << e.getErrDesc() << std::endl;
		throw e;	
	      }
	    catch (...)
	      {
		m_nb_frames = 0; // This will stop the reader
		//std::cout << "ERROR: Camera::->START_MSG --> unknown error (" 
		//      << std::hex << this->_marccd_state << std::dec << ")"
		//      << std::endl;
		throw  LIMA_HW_EXC(Error, "Camera::->START_MSG ") 
		  << DEB_HEX(this->_marccd_state);	
	      }
	    
	    DEB_TRACE() << "Camera::->START_MSG DONE";
	    //std::cout << "Camera::->START_MSG DONE." << std::endl;
	  }
	  break;
	  //-----------------------------------------------------
	case STOP_MSG:
	  {
	    DEB_TRACE() << "Camera::->STOP_MSG";

	    try 
	      {
		this->perform_abort_sequence();
	      }
	    catch (Exception e)
	      {
		//std::cout << "ERROR: Camera::->STOP_MSG --> " 
		//	  << e.getErrDesc() << std::endl;
		throw e;	
	      }
	    catch (...)
	      {
		//std::cout << "ERROR: Camera::->STOP_MSG --> unknown error (" 
		//	  << std::hex << this->_marccd_state << std::dec << ")"
		//	  << std::endl;
		throw  LIMA_HW_EXC(Error, "Camera::->STOP_MSG ") 
		  << DEB_HEX(this->_marccd_state);	
	      }
	    
	    DEB_TRACE() << "Camera::->STOP_MSG DONE";
	    //std::cout << "Camera::->STOP_MSG DONE." << std::endl;
	  }
	  break;
	  //-----------------------------------------------------
	case BACKGROUND_FRAME_MSG:
	  {
	    DEB_TRACE() << "Camera::->BACKGROUND_FRAME_MSG";
	    
	    try
	      {
		this->perform_background_frame();
	      }
	    catch (Exception e)
	      {
		//std::cout << "ERROR: Camera::->BACKGROUND_FRAME_MSG --> " 
		//	  << e.getErrDesc() << std::endl;
		throw e;	
	      }
	    catch (...)
	      {
		//std::cout << "ERROR: Camera::->BACKGROUND_FRAME_MSG --> unknown error (" 
		//	  << std::hex << this->_marccd_state << std::dec << ")"
		//	  << std::endl;
		throw  LIMA_HW_EXC(Error, "Camera::->BACKGROUND_FRAME_MSG ") 
		  << DEB_HEX(this->_marccd_state);	
	      }
	    
	    //std::cout << "Camera::->BACKGROUND_FRAME_MSG DONE." << std::endl;
	    DEB_TRACE() << "Camera::->BACKGROUND_FRAME_MSG DONE";
	  }
	  break;
	}
    }
  catch(yat::Exception& ex)	//- XE : Dangerous -> 'cause cannot be caught !!!
    {
      //std::cout << "\n\t****** HANDLE_MSG -> Error on YAT exception which desc is : " << ex.errors[0].desc << std::endl;
      //throw;
    }
  catch(...)
    {
      //std::cout << "\n\t****** HANDLE_MSG -> Error : Generic Exception caught !!!" << std::endl;
    }
}

//---------------------------
//- Camera::start()
//---------------------------
void Camera::start()
{
  DEB_MEMBER_FUNCT();
  DEB_TRACE() << "Camera::start() - ENTERING ..." ;
  //std::cout <<"Camera::start() - ENTERING ..." << std::endl;
  
  if( !this->_sock )
    throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");
  
  //- prepare msg
  yat::Message * msg = new yat::Message(START_MSG, MAX_USER_PRIORITY);
  if ( !msg )
    {
      std::cerr << "Camera::start FAILED -> yat::Message allocation FAILED!"  << std::endl;
      //- TODO : gestion erreur !???
      return;
    }
  //- don't wait till the message is processed !!
  this->post(msg);
  
  DEB_TRACE() << "Camera::start() - MARCCD DONE" ;
  //std::cout << "Camera::start() - MARCCD DONE" << std::endl;
}

//---------------------------
//- Camera::stop()
//---------------------------
void Camera::stop()
{
  DEB_MEMBER_FUNCT();
  DEB_TRACE() << "Camera::stop() - ENTERING ...";
  //std::cout << "Camera::stop() - ENTERING ..." << std::endl;
  
  if( !this->_sock )
    throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");
  
  this->_abort = true;
  //- prepare msg
  yat::Message * msg = new yat::Message(STOP_MSG, MAX_USER_PRIORITY);
  if ( !msg )
    {
      std::cerr << "Camera::stop -> yat::Message allocation FAILED!"  << std::endl;
      //- TODO : gestion erreur !???
      return;
    }
  
  //- don't wait till the message is processed !!
  this->post(msg);
  
  DEB_TRACE() << "Camera::stop() - MARCCD DONE" ;
  //std::cout << "Camera::stop() - MARCCD DONE" << std::endl;
}

//---------------------------
//- Camera::take_background_frame()
//---------------------------
void Camera::take_background_frame()
{
  DEB_MEMBER_FUNCT();
  DEB_TRACE() << "Camera::take_background_frame() - ENTERING ...";
  //std::cout << "Camera::take_background_frame() - ENTERING ..." << std::endl;

  _bgAcquired = false;

  if( !this->_sock )
    throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");
  
  
  //- prepare msg
  yat::Message * msg = new yat::Message(BACKGROUND_FRAME_MSG, MAX_USER_PRIORITY);
  if ( !msg )
    {
      std::cerr << "Camera::take_background_frame -> yat::Message allocation FAILED!"  << std::endl;
      //- TODO : gestion erreur !???
      return;
    }
  
  //- don't wait till the message is processed !!
  this->post(msg);
  
  DEB_TRACE() << "Camera::take_background_frame() - MARCCD DONE" ;
  //std::cout << "Camera::take_background_frame() - MARCCD DONE" << std::endl;
}

//---------------------------
//- Camera::prepare() //- TODO : make a yat msg ?
//---------------------------
void Camera::prepare()
{
	if( !this->_sock )
		throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");

	//- Method to take a backround image. This background will be substacted to each
	//-  taken images !
	//- TODO periodically ?! If so, period(in ms) = ?

	//- SEQUENCE :
	//- wait for marccd to not be reading

	//- send readout 1
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getImageSize(Size& size)
{
	DEB_MEMBER_FUNCT();

	if( !this->_sock )
		throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");

	std::string resp ("");
	int sizeX, sizeY;
	try
	{
	  yat::MutexLock scoped_lock(this->_lock);
	  //- get the max image size of the detector
	  resp = this->write_read("get_size");
	  
	  sscanf(resp.c_str(), "%d,%d", &sizeX, &sizeY);
	  
	  size = Size(sizeX,sizeY);
	}
	catch (...)
	{
		//- Error handling
		std::cerr << "MARCAM GET_IMAGE_SIZE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getPixelSize(double& x_size,double& y_size)
{
	DEB_MEMBER_FUNCT();
	x_size = y_size = PixelSize;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getImageType(ImageType& type)
{
	DEB_MEMBER_FUNCT();
	//-	default:
	type = Bpp16;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorType(std::string& type)
{
	DEB_MEMBER_FUNCT();
	type = this->_detector_type;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorModel(std::string& type)
{
	DEB_MEMBER_FUNCT();
	type = this->_detector_model;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setMaxImageSizeCallbackActive(bool )
{

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setTrigMode(TrigMode mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(mode);

	//- TODO : any trigger (should be internal !) or use an external one !??
	//- if external how & where to manage it !???
	try
	{
		if ( mode == IntTrig )
		{
			//- INTERNAL
			//this->Camera_->TriggerMode.SetValue( TriggerMode_Off );
			////@@@@ TODO later this mode is disabled !!
			////this->Camera_->AcquisitionFrameRateEnable.SetValue( true );
			//////////////////////////////////////////////////////////////////
		  //std::cout << "Camera::setTrigMode -> IntTrig" << std::endl;
		}
		else if ( mode == ExtGate )
		{
			//- EXTERNAL - TRIGGER WIDTH
			//this->Camera_->TriggerMode.SetValue( TriggerMode_On );
			//this->Camera_->AcquisitionFrameRateEnable.SetValue( false );
			//this->Camera_->ExposureMode.SetValue( ExposureMode_TriggerWidth );
		  //std::cout << "Camera::setTrigMode -> ExtGate" << std::endl;
		}
		else //ExtTrigSingle
		{
			//- EXTERNAL - TIMED
			//this->Camera_->TriggerMode.SetValue( TriggerMode_On );
			//this->Camera_->AcquisitionFrameRateEnable.SetValue( false );
			//this->Camera_->ExposureMode.SetValue( ExposureMode_Timed );
		  //std::cout << "Camera::setTrigMode -> else" << std::endl;
		}
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM SET_TRIG_MODE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
  DEB_MEMBER_FUNCT();
  //- TODO : for the moment and device compatibility forced to INTERNAL !!!
  mode = IntTrig;
  DEB_RETURN() << DEB_VAR1(mode);
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setExpTime(double exp_time)
{
  DEB_MEMBER_FUNCT();
  //std::cout << "Camera::setExpTime(" <<exp_time<< ")" << std::endl;
  DEB_PARAM() << DEB_VAR1(exp_time);

  // Minimal exposure time is about the minimal readout time
  // and readout time is 5s / binning (2.5 for 2x2, 1.25 for 4x4 , ...)

//   if( exp_time < 5./m_binning )
//     throw LIMA_HW_EXC(NotSupported, "Invalid time for this binning ") 
//       << DEB_VAR2(exp_time,m_binning);
  
      m_exp_time = exp_time;

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
  DEB_MEMBER_FUNCT();
  exp_time = m_exp_time;
  DEB_RETURN() << DEB_VAR1(exp_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(lat_time);

  // Check if the exposure time is enough for the current binning
  // Minimal exposure time is about the minimal readout time
  // and readout time is 5s / binning (2.5 for 2x2, 1.25 for 4x4 , ...)
  if( lat_time < 5./m_binning )
    throw LIMA_HW_EXC(NotSupported, "Invalid time for this binning ") 
      << DEB_VAR2(lat_time,m_binning);
  
      m_lat_time = lat_time;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
  DEB_MEMBER_FUNCT();
  lat_time = m_lat_time;
  DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(nb_frames);
  m_nb_frames = nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
  DEB_MEMBER_FUNCT();
  nb_frames = m_nb_frames;
  DEB_RETURN() << DEB_VAR1(nb_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getFrameRate(double& frame_rate)
{
  DEB_MEMBER_FUNCT();
  frame_rate = 123456.7;
  DEB_RETURN() << DEB_VAR1(frame_rate);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(set_roi);
    hw_roi = set_roi;

    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setRoi(const Roi& set_roi)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(set_roi);
    Roi r;    
		try
		{
			//- backup old roi, in order to rollback if error
			getRoi(r);
			if(r == set_roi) return;

			//- first reset the ROI
			Size size;
			this->getImageSize(size);

			Roi fullFrame( 0,
				0,
				static_cast<int>(size.getWidth()),
				static_cast<int>(size.getHeight())
				);

			if(set_roi.isActive() && fullFrame != set_roi)
			{
				//- then fix the new ROI
				//Camera_->Width.SetValue( set_roi.getSize().getWidth());
				//Camera_->Height.SetValue(set_roi.getSize().getHeight());
				//Camera_->OffsetX.SetValue(set_roi.getTopLeft().x);
				//Camera_->OffsetY.SetValue(set_roi.getTopLeft().y);
			}
		}
		catch(...)
		{
			DEB_ERROR() << "Marccd::Camera::setRoi -> generic exception caught !";
			throw LIMA_HW_EXC(Error, "Marccd::Camera::setRoi -> generic exception caught !");
		}
		//catch (GenICam::GenericException &e)
		//{
		//	try
		//	{
		//		//-  rollback the old roi
		//		Camera_->Width.SetValue( r.getSize().getWidth());
		//		Camera_->Height.SetValue(r.getSize().getHeight());
		//		Camera_->OffsetX.SetValue(r.getTopLeft().x);
		//		Camera_->OffsetY.SetValue(r.getTopLeft().y);
		//		// Error handling
		//	}
		//	catch (GenICam::GenericException &e2)
		//	{
		//		DEB_ERROR() << e2.GetDescription();
		//		throw LIMA_HW_EXC(Error,e2.GetDescription());
		//	}
		//	DEB_ERROR() << e.GetDescription();
		//	throw LIMA_HW_EXC(Error, e.GetDescription());
		//}        

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
		try
		{
			Size size;
			//- get image size to set ROI max
			this->getImageSize(size);
			Roi  r( static_cast<int>(0),
							static_cast<int>(0),
							static_cast<int>(size.getWidth()),
							static_cast<int>(size.getHeight())
						);

			hw_roi = r;
		}
		catch (...)
		{
			// Error handling
			DEB_ERROR() << "Marccd::Camera::getRoi -> generic exception caught !";
			throw LIMA_HW_EXC(Error, "Marccd::Camera::getRoi -> generic exception caught !");
		}    
    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::checkBin(Bin& bin)
{
	int binX = bin.getX();
	int binY = bin.getY();

	if ( binX > 8 )
		binX = 8;
	else if ( binX < 1 )
		binX = 1;
	else if ( binY > 8 )
		binY = 8;
	else if ( binY < 1 )
		binY = 1;

	bin = Bin(binX, binY);
  //DEB_RETURN() << DEB_VAR1(bin);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setBinning(const Bin &bin)
{
  std::stringstream bin_values;
  
  if( !this->_sock )
    throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");
  
  //- This is MarCCD supported bin values : "2,2" ; "3,3" ; "4,4" ; "8,8" ;
  try
    {
      if ( bin.getY() == bin.getX() )
	{
	  m_binning = bin.getX();
	  bin_values << "set_bin," << m_binning << "," << m_binning << std::ends;
	  //std::cout << "Camera::setBinning -> sending : *" << bin_values.str() << "*" << std::endl;
	  yat::MutexLock scoped_lock(this->_lock);
	  //- set the new binning values
	  this->write_read(bin_values.str());
	  //std::cout << "Camera::setBinning -> DONE" << std::endl;
	}
    }
  catch (...)
    {
      //- Error handling
      std::cerr << "MARCAM SET_BINNING -> An ... exception occurred!"  << std::endl;
      //- XE throw LIMA_HW_EXC(Error, e.GetDescription());
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getBinning(Bin& bin)
{
  
  std::cout << "\n\t\tMarccdCamera::getBinning DONE -> ENTERING ..." << std::endl;
  DEB_MEMBER_FUNCT();
  
  if( !this->_sock )
    throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");

  std::string resp ("");
  int binX, binY;
  
  try
    {
      yat::MutexLock scoped_lock(this->_lock);
      //- get the new binning values
      resp = this->write_read("get_bin");
      
      sscanf(resp.c_str(), "%d,%d", &binX, &binY);
      
      if( binX == binY )
	{
	  bin = Bin(binX, binY);
	  m_binning = binX;
	  //std::cout << "MarccdCamera::getBinning DONE -> binX = " << binX << " binY = " << binY << std::endl;
	}
      
    }
  catch (...)
    {
      // Error handling
      std::cerr << "MARCAM GET_BINNING -> An ... exception occurred!"  << std::endl;
      //- XE throw LIMA_HW_EXC(Error, e.GetDescription());
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getStatus(Camera::Status& status)
{
  //- check if connection is up
  if( !this->_sock )
    {
      status = Camera::Fault;
      //std::cout << "Camera::getStatus -> no _sock !" << std::endl;
      return;
    }
  
  size_t acquireStatus, readoutStatus, correctStatus, writingStatus, dezingerStatus;
  
  DEB_MEMBER_FUNCT();
  
  this->get_marccd_state();
  
  //size_t mar_status = TASK_STATE(this->_marccd_state);
  
  acquireStatus = TASK_STATUS(this->_marccd_state, TASK_ACQUIRE);
  readoutStatus = TASK_STATUS(this->_marccd_state, TASK_READ);
  correctStatus = TASK_STATUS(this->_marccd_state, TASK_CORRECT);
  writingStatus = TASK_STATUS(this->_marccd_state, TASK_WRITE);
  dezingerStatus= TASK_STATUS(this->_marccd_state, TASK_DEZINGER);
  
  if (!_bgAcquired)                                 m_status = Camera::Config;
  else if (this->_marccd_state == 0)                m_status = Camera::Ready;
  else if (this->_marccd_state == 7)                m_status = Camera::Fault;
  else if (this->_marccd_state == 8)                m_status = Camera::Ready;  /* This is really busy interpreting command but we don't have a status for that yet */
  else if (acquireStatus & (TASK_STATUS_EXECUTING)) m_status = Camera::Exposure;
  else if (readoutStatus & (TASK_STATUS_EXECUTING)) m_status = Camera::Readout;
  else if (correctStatus & (TASK_STATUS_EXECUTING)) m_status = Camera::Readout;/*ADStatusCorrect*/
  else if (writingStatus & (TASK_STATUS_EXECUTING)) m_status = Camera::Readout;/*ADStatusSaving*/
  if ((acquireStatus | readoutStatus | correctStatus | writingStatus | dezingerStatus) & 	TASK_STATUS_ERROR)
    m_status = Camera::Fault;
  
  status = m_status;
  
  DEB_TRACE() << "Camera::getStatus() - status = " << status;
  DEB_RETURN() << DEB_VAR1(DEB_HEX(status));
}

//-----------------------------------------------------
//
//-----------------------------------------------------
unsigned int Camera::getState()
{
  this->get_marccd_state();
  return this->_marccd_state;
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setBeamX(float X)
{
  std::stringstream cmd;
  if( !this->_sock )
    throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");
  
  try
    {
      cmd << "header,beam_x=" << X << "\n";
      yat::MutexLock scoped_lock(this->_lock);
      this->write_read(cmd.str());
    }
  catch (...)
    {
      //- Error handling
      std::cerr << "MARCAM SET BEAM X -> An ... exception occurred!"  << std::endl;
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setBeamY(float Y)
{
  std::stringstream cmd;
  if( !this->_sock )
    throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");
  
  try
    {
      cmd << "header,beam_y=" << Y << "\n";
      yat::MutexLock scoped_lock(this->_lock);
      this->write_read(cmd.str());
    }
  catch (...)
    {
      //- Error handling
      std::cerr << "MARCAM SET BEAM Y -> An ... exception occurred!"  << std::endl;
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setDistance(float D)
{
  std::stringstream cmd;
  if( !this->_sock )
    throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");
  
  try
    {
      cmd << "header,detector_distance=" << D << "\n";
      yat::MutexLock scoped_lock(this->_lock);
      this->write_read(cmd.str());
    }
  catch (...)
    {
      //- Error handling
      std::cerr << "MARCAM SET DISTANCE -> An ... exception occurred!"  << std::endl;
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setWavelength(float W)
{
  std::stringstream cmd;
  if( !this->_sock )
    throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");
  
  try
    {
      cmd << "header,source_wavelength=" << W << "\n";
      yat::MutexLock scoped_lock(this->_lock);
      this->write_read(cmd.str());
    }
  catch (...)
    {
      //- Error handling
      std::cerr << "MARCAM SET WAVELENGTH -> An ... exception occurred!"  << std::endl;
    }
}

//-----------------------------------------------------
// - INTERNAL METHODS
//-----------------------------------------------------
void Camera::disconnect()
{
  DEB_MEMBER_FUNCT();
  //- check if connection is up
  if( !this->_sock )
    {
      //status = Camera::Fault;
      //std::cout << "Camera::disconnect -> no _sock !" << std::endl;
      return;
    }
  
  try
    {
      this->_sock->disconnect();
      //std::cout << "INFO::Camera::disconnect(): DONE !" << std::endl;
    }
  catch(yat::SocketException & ySe)
    {
      //std::cout << "ERROR::Camera::disconnect -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
    }
  catch(...)
    {
      //std::cout << "ERROR::Camera::disconnect -> [...] Exception." << std::endl;
    }
}

void Camera::connect()
{
  DEB_MEMBER_FUNCT();
  //std::cout << "Camera::connect -> ENTERING ..." << std::endl;
  //- check if connection is up
  if( !this->_sock )
    {
      //status = Camera::Fault;
      //std::cout << "Camera::connect -> no _sock !" << std::endl;
      return;
    }
  
  try
    {
      this->_sock->set_option(yat::Socket::SOCK_OPT_KEEP_ALIVE, 1);
      this->_sock->set_option(yat::Socket::SOCK_OPT_NO_DELAY, 1);
      this->_sock->set_option(yat::Socket::SOCK_OPT_OTIMEOUT, 0);
      this->_sock->set_option(yat::Socket::SOCK_OPT_ITIMEOUT, 0);
      //std::cout << "Camera::connect -> OPT set" << std::endl;
      
      yat::Address addr(this->_camera_ip, this->_port_num);
      //std::cout << "INFO::Camera::connect(): made Address... !" << std::endl;
      
      this->_sock->connect(addr);
      //std::cout << "INFO::Camera::connect(): connect DONE." << std::endl;
    }
  //- TODO : on error -> sock = NULL
  catch(yat::SocketException & ySe)
    {
      //std::cout << "ERROR::Camera::connect -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
      throw;
    }
  catch(...)
    {
      //std::cout << "ERROR::Camera::connect -> [...] Exception." << std::endl;
      DEB_ERROR() << "ERROR::Camera::connect -> [...] Exception." ;
      throw;
    }
}

//-----------------------------------------------------
// - read : returns Marccd command response
//-----------------------------------------------------
std::string Camera::read()
{
	//- check if connection is up
	if( !this->_sock )
	{
	  //status = Camera::Fault;
	  //std::cout << "Camera::read -> no _sock !" << std::endl;
		throw;
	}
	
	std::string response("");
	try
	{
	//- TODO : CHECK if sock = NULL
	  this->_sock->receive(response);
	  //std::cout << "INFO::Camera::read(): response = " << response << std::endl;
	}
	catch(yat::SocketException & ySe)
	{
	  //std::cout << "ERROR::Camera::read -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
		throw;
	}
	catch(...)
	{
	  //std::cout << "ERROR::Camera::read -> [...] Exception." << std::endl;
		throw;
	}

	return response;
}

//-----------------------------------------------------
// - write : sends Marccd command
//-----------------------------------------------------
void Camera::write(std::string cmd_to_send)
{
  DEB_MEMBER_FUNCT();
	//- check if connection is up
	if( !this->_sock )
	{
	  //status = Camera::Fault;
	  //std::cout << "Camera::write -> no _sock !" << std::endl;
		return;
	}
	
	try
	{
	  size_t data_size = cmd_to_send.size() + 1; //- +1 for '\0' character
	  //- TODO : CHECK if sock = NULL
	  this->_sock->send(cmd_to_send.c_str(), data_size);

	  //std::cout << "\n\nINFO::Camera::write(): cmd_to_be_send = " << cmd_to_send << " WRITTEN." << std::endl;
	}
	catch(yat::SocketException & ySe)
	{
	  //std::cout << "ERROR::Camera::write -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
	}
	catch(...)
	{
	  DEB_ERROR() << "ERROR::Camera::write -> [...] Exception.";
	  //std::cout << "ERROR::Camera::write -> [...] Exception." << std::endl;
	}
}

//-----------------------------------------------------
// - write_read : Sends a command and reads response
//-----------------------------------------------------
std::string Camera::write_read(std::string cmd_to_send)
{
	//- check if connection is up
	if( !this->_sock )
	{
	  //status = Camera::Fault;
	  //std::cout << "Camera::write_read -> no _sock !" << std::endl;

	  throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");
	}
	
	std::string response("");
	//- send command
	this->write(cmd_to_send);
	//- check if command needs a response
	if ( cmd_to_send.find("get_") != std::string::npos )
		response = this->read();

	return response;
}

//-----------------------------------------------------
// - perform_acquisition_sequence : manage a Marccd acquisition
//-----------------------------------------------------
void Camera::perform_acquisition_sequence()
{

  _first_image = _image_number;
  int step = 0;
  time_t sec = 0;
  long  nsec = 0;

  struct timespec now, exposure, latency;
  clock_gettime(CLOCK_MONOTONIC,&now);
  clock_gettime(CLOCK_MONOTONIC,&exposure); 
  clock_gettime(CLOCK_MONOTONIC,&latency);
 
  this->_stop_sequence_finished = false;
  this->_full_img_name = "";

  int laststep = 0;
  while ( _image_number < _first_image + m_nb_frames && !this->_abort )
    {
      if (laststep != step)
	{
	  laststep = step;
	  clock_gettime(CLOCK_MONOTONIC,&now);
	  std::cout << "Step " << step << " (t " 
	    //<< (now.tv_sec + (double) now.tv_nsec/1000000000 )
		    << now.tv_sec << "." 
		    << std::setfill('0') << std::setw(9) << now.tv_nsec
		    << ")\n";
	}
      
      switch (step)
	{
	case 0: //- Prepare acquisition
	  step++;
	  break;

	case 1: //- Wait for Marccd to not be acquiring 
	  this->get_marccd_state();
	  if (TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_ERROR) 
	      || TEST_TASK_STATUS(this->_marccd_state,TASK_READ,TASK_STATUS_ERROR) 
	      || TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATUS_ERROR))
	    {
	      throw LIMA_HW_EXC(Error, "Camera::perform_start_sequence (Wait before start) ")
		<< DEB_HEX(this->_marccd_state);		    
	    }

	  if (TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_EXECUTING))
	    break;
	  step++;
	  break;

	case 2: //- Wait for the latency time
	  clock_gettime(CLOCK_MONOTONIC,&now);
	  if (latency.tv_sec + (double) latency.tv_nsec /1000000000.0 
	      - now.tv_sec - (double) now.tv_nsec /1000000000.0 > 0.2)
	    {
	      sleep(0.1);
	      break;
	    }
	  step++;
	  break;
	  
	case 3: //- Send start cmd
	  {
	    std::string cmd_to_send("start");

	    yat::MutexLock scoped_lock(this->_lock);
	    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&latency,NULL); 
	    clock_gettime(CLOCK_MONOTONIC,&now);
	    this->write_read(cmd_to_send);
	  }
	  // Define when the exposure time finish
	  sec = (int) m_exp_time;
	  nsec= (m_exp_time - sec)*1000000000;
	  if (nsec + now.tv_nsec >= 1000000000)
	    {
	      nsec -= 1000000000;
	      sec++;
	    }
	  exposure.tv_sec  = now.tv_sec + sec;
	  exposure.tv_nsec = now.tv_nsec + nsec;
	  // Define when the latency time finish
	  sec = (int) m_lat_time;
	  nsec= (m_lat_time - sec)*1000000000;
	  if (nsec + exposure.tv_nsec >= 1000000000)
	    {
	      nsec -= 1000000000;
	      sec++;
	    }
	  latency.tv_sec  = exposure.tv_sec + sec;
	  latency.tv_nsec = exposure.tv_nsec + nsec;

	  std::cout << "Start ->"
		    << now.tv_sec << "." 
		    << std::setfill('0') << std::setw(9) << now.tv_nsec
		    << ")\n";
	  std::cout << "Stop  ->"
		    << exposure.tv_sec << "." 
		    << std::setfill('0') << std::setw(9) << exposure.tv_nsec
		    << ")\n";
	  std::cout << "Lat   ->"
		    << latency.tv_sec << "." 
		    << std::setfill('0') << std::setw(9) << latency.tv_nsec
		    << ")\n";


	  step++;
	  break;
	  
	case 4: //- Wait for acquisition to be started
	  this->get_marccd_state();
	  if (TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_ERROR) 
	      || TEST_TASK_STATUS(this->_marccd_state,TASK_READ,TASK_STATUS_ERROR) 
	      || TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATUS_ERROR))
	    {
	      throw LIMA_HW_EXC(Error, "Camera::perform_start_sequence (Wait before start) ")
		<< DEB_HEX(this->_marccd_state);		    
	    }
	  
	  if (!TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_EXECUTING))
	    break;
	  step++;
	  break;
	  
	case 5: //- Wait for the exposure time 
	  clock_gettime(CLOCK_MONOTONIC,&now);
	  if (exposure.tv_sec + (double) exposure.tv_nsec /1000000000.0 
	      - now.tv_sec - (double) now.tv_nsec /1000000000.0 > 0.2)
	    {
	      sleep(0.1);
	      break; 
	    }
	  step++;
	  break;

	case 6: //- Send readout cmd with a specific file name
	  {
	    std::string cmd_to_send;
	    if (!_saveBG)
	      {
		cmd_to_send = "readout,0,";
	      }
	    else
	      {
		cmd_to_send = "readout,1,";
	      }
	    this->_full_img_name = this->_image_path + this->_image_name 
	      + "_" + yat::XString<size_t>::to_string(this->_image_number);
	    cmd_to_send += this->_full_img_name;
	    
	    yat::MutexLock scoped_lock(this->_lock);
	    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&exposure,NULL); 
	    this->write_read(cmd_to_send);

	    _saveBG = false;
	  }
	  step++;
	  break;

	case 7: //- Wait for Marccd until writing has finished
	  this->get_marccd_state();
	  if (TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_ERROR) 
	      || TEST_TASK_STATUS(this->_marccd_state,TASK_READ,TASK_STATUS_ERROR) 
	      || TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATUS_ERROR))
	    {
	      throw LIMA_HW_EXC(Error, "Camera::perform_stop_sequence (Wait for writting)")
		<< DEB_HEX(this->_marccd_state);		    
	    }
	  
	  if (TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATE_WRITING))
	    break;
	  
	  this->_image_number++;
	  this->_stop_sequence_finished = true;
	  step++;
	  break;

	default:
	  step = 0;
	  
	} //end switch 
    } // end while

}


//-----------------------------------------------------
// - perform_background_frame : sequence to take a background (dark) frame
//-----------------------------------------------------
void Camera::perform_background_frame()
{
  //std::cout << "\n\n\nCamera::perform_background_frame <- " << std::endl;
  
  //- wait for detector NOT be reading
  //std::cout << "Wait for READ + EXECUTING" << std::endl;

  do
    {
      this->get_marccd_state();
      if (TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_ERROR) 
	  || TEST_TASK_STATUS(this->_marccd_state,TASK_READ,TASK_STATUS_ERROR) 
	  || TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATUS_ERROR))
	{
	  throw LIMA_HW_EXC(Error, "Camera::perform_background_frame (Wait before readout,1) ")
	    << DEB_HEX(this->_marccd_state);		    
	}
      //std::cout << "Camera::perform_background_frame -> wait for detector NOT be reading, state = " << this->_marccd_state << std::endl;
    }while(TEST_TASK_STATUS(this->_marccd_state,TASK_READ,TASK_STATUS_EXECUTING));
  //std::cout << "Done for READ + EXECUTING" << std::endl;
  //std::cout << "\t **** Wait for Marccd to not be acquiring DONE." << std::endl;
  
  //- Send readout 1 => read data into background frame storage
  std::string cmd_to_send("readout,1");
  
  {
    yat::MutexLock scoped_lock(this->_lock);
    this->write_read(cmd_to_send);
  }
  
  // Wait for idle?
  this->get_marccd_state();
  while ( this->_marccd_state )
    {
      this->get_marccd_state();
      if (TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_ERROR) 
	  || TEST_TASK_STATUS(this->_marccd_state,TASK_READ,TASK_STATUS_ERROR) 
	  || TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATUS_ERROR))
	{
	  throw LIMA_HW_EXC(Error, "Camera::perform_background_frame (Wait before readout,2) ")
	    << DEB_HEX(this->_marccd_state);		    
	}
    }
  
  //- Send readout 2 => read data into system scratch storage
  cmd_to_send = "readout,2";
  {
    yat::MutexLock scoped_lock(this->_lock);
    this->write_read(cmd_to_send);
  }
  this->get_marccd_state();
  while ( this->_marccd_state )
    {
      this->get_marccd_state();
      if (TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_ERROR) 
	  || TEST_TASK_STATUS(this->_marccd_state,TASK_READ,TASK_STATUS_ERROR) 
	  || TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATUS_ERROR))
	{
	  throw LIMA_HW_EXC(Error, "Camera::perform_background_frame (Wait before dezinger,1) ")
	    << DEB_HEX(this->_marccd_state);		    
	}
    }
  //std::cout << "\n\t **** READOUT 2 DONE." << std::endl;
  
  //- Send dezinger 1 => use and store into the current background frame
  cmd_to_send = "dezinger,1";
  {
    yat::MutexLock scoped_lock(this->_lock);
    this->write_read(cmd_to_send);
  }
  //this->get_marccd_state();
  
  //std::cout << "Wait for DEZINGER + EXECUTING" << std::endl;
  do
    {
      this->get_marccd_state();
      if (TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_ERROR) 
	  || TEST_TASK_STATUS(this->_marccd_state,TASK_READ,TASK_STATUS_ERROR) 
	  || TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATUS_ERROR))
	{
	  throw LIMA_HW_EXC(Error, "Camera::perform_background_frame (Wait for dezinger) ")
	    << DEB_HEX(this->_marccd_state);		    
	}
      //std::cout << "Camera::perform_background_frame -> dezinger STATE : " << this->_marccd_state << std::endl;
    }while(TEST_TASK_STATUS(this->_marccd_state,TASK_DEZINGER,TASK_STATUS_EXECUTING));

  _bgAcquired = true;

  //std::cout << "Done for DEZINGER + EXECUTING" << std::endl;

  //std::cout << "Camera::perform_background_frame -> readout_2 FINISHED : state = " << this->_marccd_state << std::endl;
  //std::cout << "Camera::perform_background_frame -> dezinger,1, state = " << this->_marccd_state << std::endl;
  
  
  //std::cout << "Camera::perform_background_frame -> \n\n" << std::endl;
  
  //do
  //{
  //	this->get_marccd_state();
  //	std::cout << "Camera::perform_background_frame -> CHECKING MARCCD STATE : " << this->_marccd_state << std::endl;
  //}while(TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATE_WRITING));
}

//-----------------------------------------------------
// - perform_abort_sequence : abort Marccd acquisiion
//-----------------------------------------------------
void Camera::perform_abort_sequence()
{
  DEB_MEMBER_FUNCT();
  //std::cout << "Camera::perform_abort_sequence <- " << std::endl;
    
  //---------------------------------------
  //-		TELL MARCCD ABORT ACQUIRING
  //---------------------------------------
  //- Send abort cmd
  // 5./m_binning is the readout time in seconds
  clock_t wait = 5./m_binning * CLOCKS_PER_SEC + clock();
  {
    std::string cmd_to_send("abort");
    yat::MutexLock scoped_lock(this->_lock);
    this->write_read(cmd_to_send);
  }
  // wait for abort the same time as readout time
  while (wait > clock());
  
  {
    // Make a dummy reading to reset a any reading error
    std::string cmd_read("readout,0");
    yat::MutexLock scoped_lock(this->_lock);
    this->write_read(cmd_read);
  }
  DEB_PARAM() << " **** ABORT SENT !!!" ;
  //std::cout << "\t **** ABORT SENT !!!" << std::endl;  
  
  //---------------------------------------
  //-		SHUTTER CONTROLLED IN DS -> TODO : to be implemented !!!
  //---------------------------------------
  //std::cout << "Camera::perform_abort_sequence -> " << std::endl;
  this->_stop_sequence_finished = true;
  this->_abort = false;
}

//-----------------------------------------------------
void Camera::get_marccd_state()
{
	if( !this->_sock )
		throw LIMA_HW_EXC(Error, "No communication opened with Marccd.");

	std::string stateStr("");
	//- get detector state string value
	{
	  yat::MutexLock scoped_lock(this->_lock);
	  try
	    {
	      stateStr = this->write_read("get_state");
	    }
	  catch(yat::Exception &ye)
	    {
	      std::cerr << "ERROR::Camera::get_marccd_state() caught YAT Exception [" << ye.errors[0].desc << "]" << std::endl;
	    }	  
	  catch(...)
	    {
	      std::cerr << "ERROR::Camera::get_marccd_state() [...] Exception." << std::endl;
	    }
	}

	//- convert state string to numeric val
	char data_to_conv[stateStr.size()+1];

	::strncpy(data_to_conv, stateStr.c_str(), stateStr.size());

	this->_marccd_state  = this->convertStringToInt(data_to_conv);
}

//-----------------------------------------------------
// - setImagePath
//-----------------------------------------------------
void Camera::setImagePath(const std::string& imgPath)
{
  this->_image_path = imgPath;
}

//-----------------------------------------------------
// - getImagePath
//-----------------------------------------------------
const std::string& Camera::getImagePath(void)
{
  return this->_image_path;
}

//-----------------------------------------------------
// - setImageFileName
//-----------------------------------------------------
void Camera::setImageFileName(const std::string& imgName)
{
  this->_image_name = imgName;
}

//-----------------------------------------------------
// - getImageFileName
//-----------------------------------------------------
const std::string& Camera::getImageFileName(void)
{
  return this->_image_name;
}

//-----------------------------------------------------
// - getFullImgName
//-----------------------------------------------------
const std::string& Camera::getFullImgName()
{
  //- image name = full path + image name + "_" + image index
  //std::cout << "\t\tCamera::getFullImgName() = " << this->_full_img_name << std::endl;
  return this->_full_img_name;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::is_stop_sequence_finished() 
{
  return this->_stop_sequence_finished;
}

void Camera::setImageIndex(int newImgIdx) 
{
  this->_image_number = newImgIdx;
}

int Camera::getImageIndex() 
{
  return this->_image_number;
}

int Camera::getFirstImage() 
{
  return this->_first_image;
}

void Camera::saveBG(bool BG)
{
  _saveBG = BG;
}

//-----------------------------------------------------
//============================================================
//Camera::convertStringToInt
//============================================================
int Camera::convertStringToInt( char* text )
{
	int intval = 0;

	if (sscanf(text, "0x%x", &intval) > 0)
		return strtoul(text, NULL, 16);
	else
		return atoi( text );
}

