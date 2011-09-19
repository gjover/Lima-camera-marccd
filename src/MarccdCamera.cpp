#include "MarccdCamera.h"
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include <yat/utils/XString.h>


using namespace lima;
using namespace lima::Marccd;
//using namespace std;

//- Define NaN value
const double _NaN_ = ::sqrt(-1.);

//- User task messages
const size_t  START_MSG			=	(yat::FIRST_USER_MSG + 100);
const size_t  STOP_MSG			=	(yat::FIRST_USER_MSG + 101);
const size_t  GET_IMAGE_MSG	=	(yat::FIRST_USER_MSG + 200);
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

// Buffers for grabbing
//static const uint32_t c_nBuffers = 1;

//---------------------------
//- Ctor
//---------------------------
Camera::Camera(const std::string& camera_ip, size_t port_num, const std::string& img_path, const std::string& img_name, const std::string& img_dir_watcher)
: yat::Task(),
	_sock(0),
	_camera_ip(camera_ip),
	_port_num(port_num),
	_image_path(img_path),
	_image_name(img_name),
  _image_dir_watcher_path(img_dir_watcher),
	_marccd_state(TASK_STATE_IDLE),
	m_status(Camera::Unknown),
	_image_number(0),
	_stop_already_done(false),
	_detector_model(""),
	_detector_type("")
{
	DEB_CONSTRUCTOR();
 
	DEB_TRACE() << "Camera::Camera() - ENTERING ...";

	try
	{
    //this->enable_periodic_msg( true );

		_stop_already_done = true;
		m_status = Camera::Ready;		
		// Create the transport layer object needed to enumerate or
		// create a camera object of type Camera_t::DeviceClass()
		std::cout << "Create a Camera object of type Camera_t::DeviceClass()" << std::endl;

		_detector_type  = "MARCCD" ;
		_detector_model = "SX 165";

    //- internal YAT stuff cooking
    yat::Socket::init();
    try
    {
      _sock = new yat::ClientSocket ();
    }
    catch (yat::Exception &ye)
    {
			std::cout << "Camera::Camera : Camera::Camera initialization failed caught DevFailed trying to create yat::ClientSocket" << std::endl;
      //this->status_str = "Camera::Camera : device initialization failed caught DevFailed trying to create yat::Socket\n";
			_sock = 0;
      std::cout << " caught YAT Exception [" << ye.errors[0].desc << "]" << std::endl;
      return;
    }
    catch (...)
    {
      std::cout << "Camera::Camera : Camera initialization failed caught ... trying to create yat::ClientSocket" << std::endl;
      //this->status_str = "Camera::init_device : device initialization failed caught ... trying to create yat::Socket\n";
			_sock = 0;
      return;
    }

    //- try to connect to Marccd ethernet server HW
    try
    {
      this->connect();
    }
    catch (yat::Exception &ye)
    {
      std::cout << "Camera::Camera : could not connect to " << this->_camera_ip <<  " caught DevFailed trying to connect" << std::endl;
      //this->status_str = "Camera::init_device : could not connect to " + url + " caught DevFailed trying to connect\n";
      std::cout << " caught YAT Exception [" << ye.errors[0].desc << "]" << std::endl;
      return;
    }
    catch (...)
    {
      std::cout << "Camera::Camera : could not connect to " << this->_camera_ip <<  " caught ... trying to connect" << std::endl;
      //this->status_str = "Camera::init_device : could not connect to " + url + " caught ... trying to connect\n";
      return;
    }
	
		std::cout  << "Create the Camera object attached to ip address : " << this->_camera_ip << std::endl;
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCCD CTOR -> An ... exception occurred!" << std::endl;
		//Pylon::PylonTerminate( );
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}	
	std::cout <<"Camera::Camera() - DONE" << std::endl;	
}

//---------------------------
//- Dtor
//---------------------------
Camera::~Camera()
{
	DEB_DESTRUCTOR();
	std::cout <<"Camera::~Camera() - ENTERING ..." << std::endl;	
	try
	{
		//- abort acquisition
		this->write_read("abort");

    //-  Delete device allocated objects
    this->disconnect ();
		std::cout << "Camera::~Camera -> disconnect ... " << std::endl;
    
		//- yat internal cooking
    yat::Socket::terminate();
		std::cout << "Camera::~Camera -> terminate ... " << std::endl;
	}
	//- TODO : catch all exceptions !
	catch (...)
	{
		// Error handling
		std::cerr << "MARCCD DTOR -> An ... exception occurred!"  << std::endl;
	}

	std::cout <<"Camera::~Camera() - MARCCD DONE" << std::endl;		
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::handle_message( yat::Message& msg )  throw( yat::Exception )
{
  try
  {
    switch ( msg.type() )
    {
      //-----------------------------------------------------	
      case yat::TASK_INIT:
      {
        std::cout << "Camera::->TASK_INIT" << std::endl;          
				
				this->enable_periodic_msg(true);          
				this->set_periodic_msg_period( kPERIODIC_MSG_PERIOD );

				std::cout << "Camera::->TASK_INIT DONE." << std::endl;          
      }
      break;
      //-----------------------------------------------------    
      case yat::TASK_EXIT:
      {
        std::cout << "Camera::->TASK_EXIT DONE." << std::endl;                
      }
      break;
      //-----------------------------------------------------    
      case yat::TASK_TIMEOUT:
      {
				std::cout << "Camera::->TASK_TIMEOUT DONE." << std::endl;       
      }
      break;
			//- TASK_PERIODIC ===================
			case yat::TASK_PERIODIC:
			{
				//std::cout << "\n\t******Camera::handle_message entering handling of TASK_PERIODIC msg" << std::endl;

				//- code relative to the task's periodic job goes here
				//yat::Timer t;
				
				//yat::MutexLock scoped_lock(this->_lock);        
				
				this->get_marccd_state();
				
				//this->getStatus(mar_status);
				//- TODO : update Marccd state/status here !???? (period value = ?)

				//std::cout << "\t********** TASK_PERIODIC COMPLETED -> marccd status = " << this->_marccd_state << std::endl;
				//std::cout << "**********\tTIME TO COMPLETE TASK_PERIODIC : " << t.elapsed_msec() << " milliseconds \n" << std::endl;
			}
			break;
			//- USER_DEFINED_MSG ================
      case START_MSG:	
      {
				std::cout << "Camera::->START_MSG <-" << std::endl;
				
				_stop_already_done = false;
				
				this->perform_start_sequence();
        				
				std::cout << "Camera::->START_MSG DONE." << std::endl;
				//this->post(new yat::Message(DLL_GET_IMAGE_MSG), kPOST_MSG_TMO);
      }
      break;
      //-----------------------------------------------------
      case STOP_MSG:
      {
				std::cout << "Camera::->STOP_MSG <-" << std::endl;
				
				this->perform_stop_sequence();

				//this->write_read("abort");
				std::cout << "Camera::->STOP_MSG DONE." << std::endl;
				//FreeImage();
      }
      break;
      //-----------------------------------------------------
      case GET_IMAGE_MSG:
      {
				std::cout << "Camera::->GET_IMAGE_MSG" << std::endl;
				
				this->GetImage();
				
				std::cout << "Camera::->GET_IMAGE_MSG DONE." << std::endl;
      }
      break;	
      //-----------------------------------------------------
      case BACKGROUND_FRAME_MSG:
      {
				std::cout << "Camera::->BACKGROUND_FRAME_MSG" << std::endl;
				
				this->perform_background_frame();
				
				std::cout << "Camera::->BACKGROUND_FRAME_MSG DONE." << std::endl;
      }
      break;	
    }
  }
  catch(yat::Exception& ex)	//- XE : Dangerous -> 'cause cannot be caught !!!
  {
		std::cout << "\n\t****** HANDLE_MSG -> Error on YAT exception which desc is : " << ex.errors[0].desc << std::endl;
		//throw;
  }
  catch(...)
  {
		std::cout << "\n\t****** HANDLE_MSG -> Error : Generic Exception caught !!!" << std::endl;
  }
}

//---------------------------
//- Camera::start()
//---------------------------
void Camera::start()
{
	DEB_MEMBER_FUNCT();
	std::cout <<"Camera::start() - ENTERING ..." << std::endl;

	//this->_image_number = 0;
	
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
	
	std::cout << "Camera::start() - MARCCD DONE" << std::endl;
}

//---------------------------
//- Camera::stop()
//---------------------------
void Camera::stop()
{
	DEB_MEMBER_FUNCT();
	std::cout << "Camera::stop() - ENTERING ..." << std::endl;
	
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
	
	std::cout << "Camera::stop() - MARCCD DONE" << std::endl;
}

//---------------------------
//- Camera::stop()
//---------------------------
void Camera::take_background_frame()
{
	DEB_MEMBER_FUNCT();
	std::cout << "Camera::take_background_frame() - ENTERING ..." << std::endl;
	
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
	
	std::cout << "Camera::take_background_frame() - MARCCD DONE" << std::endl;
}

//---------------------------
//- Camera::prepare() //- TODO : make a yat msg ?
//---------------------------
void Camera::prepare()
{
	//- Method to take a backround image. This background will be substacted to each
	//-  taken images !
	//- TODO periodically ?! If so, frequence = ?

	//- SEQUENCE :
	//- wait for marccd to not be reading

	//- send readout 1

	//- wait for marccd to not be reading

	//- send readout 2

	//- wait for marccd to not be reading

	//- dezinger 1
}

//---------------------------
//- Camera::FreeImage()
//---------------------------
void Camera::FreeImage()
{
	try
	{
		//m_image_number = 0;
		//m_status = Camera::Ready;
		//if(!_stop_already_done)
		//{
		//	_stop_already_done = true;
		//	// Get the pending buffer back (You are not allowed to deregister
		//	// buffers when they are still queued)
		//	StreamGrabber_->CancelGrab();
	
		//	// Get all buffers back
		//	for (GrabResult r; StreamGrabber_->RetrieveResult(r););
		//	
		//	// Stop acquisition
		//	std::cout <<"Stop acquisition" << std::endl;
		//	Camera_->AcquisitionStop.Execute();
		//	
		//	// Clean up
		//
		//	// You must deregister the buffers before freeing the memory
			std::cout << "Must deregister the buffers before freeing the memory" << std::endl;
		//	for (vector<CGrabBuffer*>::iterator it = BufferList_.begin(); it != BufferList_.end(); it++)
		//	{
		//		StreamGrabber_->DeregisterBuffer((*it)->GetBufferHandle());
		//		delete *it;
		//		*it = NULL;
		//	}
		//	BufferList_.clear();
		//	// Free all resources used for grabbing
			std::cout << "Free all resources used for grabbing" << std::endl;
		//	StreamGrabber_->FinishGrab();
		//}
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM FREE_IMAGE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}	
}
		
//---------------------------
//- Camera::GetImage()
//---------------------------
void Camera::GetImage()
{
/***** DONE BY DirectorWacher in Reader.

	//- pointer which will receive the image data (from file)
	std::ifstream filestr(this->_previous_img_path.c_str());
	std::filebuf *pbuf;

std::cout << "Camera::GetImage() <- ..." << std::endl;
	if ( !filestr )
	{
		std::cout << "Camera::GetImage() -> FAILED TO OPEN : "<< this->_current_img_path << " image file !!!" << std::endl;
		return;
	}
****/

	try
	{
		m_nb_frames = 11;

		do
		{
			this->get_marccd_state();
			std::cout << "Camera::GetImage -> CHECKING MARCCD STATE " << std::endl;
		}while(TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATE_WRITING));

		//if ( this->_previous_img_path == this->_current_img_path )
		//{
		//	std::cout << "Camera::GetImage -> _previous_img_path == _current_img_path !!!" << std::endl;
		//	return;
		//}

		//- get image size
		//this->getImageSize(imgSize);

		//- allocate pImage
		//pImage = new uint32_t[imgSize.getWidth()*imgSize.getHeight() * DATA_SIZE];	//- image data size is 16bits
		
		//- Lock sur le fichier
		yat::MutexLock scoped_lock(this->_lock);

		//- TODO : check _previous_img_path is NOT NULL
		//- TODO : check _previous_img_path has not be previously read, if so return previous img



		//this->_previous_img_path = this->_current_img_path;
std::cout << "Camera::GetImage() -> DONE !" << std::endl;
	}
	catch (...)
    {
        // Error handling
			std::cerr << "MARCAM GET_IMAGE -> An ... exception occurred!"  << std::endl;
        //- XE throw LIMA_HW_EXC(Error, e.GetDescription());
				std::cout << "Camera::GetImage() -> ERROR : failed to read image from file : \n"
									<< "****** $" << this->_image_path <<this->_image_name<< "$" << std::endl;
    }			
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getImageSize(Size& size)
{
	DEB_MEMBER_FUNCT();
	
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
void Camera::getPixelSize(double& size)
{
	DEB_MEMBER_FUNCT();
	size = PixelSize;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getImageType(ImageType& type)
{
	DEB_MEMBER_FUNCT();
	try
	{
		//PixelSizeEnums ps = Camera_->PixelSize.GetValue();
		//switch( ps )
		//{
		//	case PixelSize_Bpp8:
		//		type= Bpp8;
		//	break;
		//
		//	case PixelSize_Bpp12:
		//		type= Bpp12;
		//	break;
		//
		//	case PixelSize_Bpp16: //- this is in fact 12 bpp inside a 16bpp image
		//		type= Bpp16;
		//	break;
		//
		//	default:
				type = Bpp16;
		//	break;
		//}
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM GET_IMAGE_TYPE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}		

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorType(std::string& type)
{
	DEB_MEMBER_FUNCT();
	type = this->_detector_type;
	return;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorModel(std::string& type)
{
	DEB_MEMBER_FUNCT();
	type = this->_detector_model;
	return;		
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setMaxImageSizeCallbackActive(bool cb_active)
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
std::cout << "Camera::setTrigMode -> IntTrig" << std::endl;
		}
		else if ( mode == ExtGate )
		{
		  //- EXTERNAL - TRIGGER WIDTH
		  //this->Camera_->TriggerMode.SetValue( TriggerMode_On );
		  //this->Camera_->AcquisitionFrameRateEnable.SetValue( false );
		  //this->Camera_->ExposureMode.SetValue( ExposureMode_TriggerWidth );
std::cout << "Camera::setTrigMode -> ExtGate" << std::endl;
		}		
		else //ExtTrigSingle
		{
		  //- EXTERNAL - TIMED
		  //this->Camera_->TriggerMode.SetValue( TriggerMode_On );
		  //this->Camera_->AcquisitionFrameRateEnable.SetValue( false );
		  //this->Camera_->ExposureMode.SetValue( ExposureMode_Timed );
std::cout << "Camera::setTrigMode -> else" << std::endl;
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
	try
	{
		//if (this->Camera_->TriggerMode.GetValue() == TriggerMode_Off)
		//- TODO : for the moment and device compatibility forced to INTERNAL !!!
		mode = IntTrig;
		//else if (this->Camera_->ExposureMode.GetValue() == ExposureMode_TriggerWidth)
		//	mode = ExtGate;
		//else //ExposureMode_Timed
		//	mode = ExtTrigSingle;
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM GET_TRIG_MODE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}		
    DEB_RETURN() << DEB_VAR1(mode);
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setExpTime(double exp_time_ms)
{
	DEB_MEMBER_FUNCT();
	std::cout << "Camera::setExpTime(" <<exp_time_ms<< ")" << std::endl;
	DEB_PARAM() << DEB_VAR1(exp_time_ms);

	try
	{
		//- TODO : exposure managed externally !!

		//Camera_->ExposureTimeBaseAbs.SetValue(1E3 * exp_time_ms);
		
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM SET_EXP_TIME -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}		
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time_ms)
{
	DEB_MEMBER_FUNCT();
	try
	{
		//- TODO : to be deleted !????

		//- exposure time controlled externally !!!
		exp_time_ms = _NaN_;
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM GET_EXP_TIME -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}			
	DEB_RETURN() << DEB_VAR1(exp_time_ms);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(lat_time);
	/////@@@@ TODO if necessary
	lat_time = 0.1;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
	DEB_MEMBER_FUNCT();
	/////@@@@ TODO if necessary
	lat_time = _NaN_;
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
	try
	{
		frame_rate = 123456.7;
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM GET_FRAME_RATE -> An ... exception occurred!" << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}		
	DEB_RETURN() << DEB_VAR1(frame_rate);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setBinning(const Bin &bin)
{
	std::string cmd_to_send("set_bin,");
	std::stringstream bin_values;

	//- This is MarCCD supported bin values !
	//switch(bin)
	//{
	//case 0 :	cmd_to_send += "1,1";
	//	break;
	//case 1 :	cmd_to_send += "2,2";
	//	break;
	//case 2 :	cmd_to_send += "3,3";
	//	break;
	//case 3 :	cmd_to_send += "4,4";
	//	break;
	//case 4 :	cmd_to_send += "8,8";
	//	break;
	//}

	try
	{
		if ( bin.getY() == bin.getX() )
		{
			bin_values << "set_bin," << bin.getY() << "," << bin.getX() << std::ends;
			std::cout << "Camera::setBinning -> sending : *" << bin_values.str() << "*" << std::endl;
			yat::MutexLock scoped_lock(this->_lock);        
			//- set the new binning values
			this->write_read(bin_values.str());
			std::cout << "Camera::setBinning -> DONE" << std::endl;
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
	DEB_MEMBER_FUNCT();
	
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
			std::cout << "MarccdCamera::getBinning DONE" << std::endl;
			//if( binX == 1 )
			//	bin = 0;
			//if( binX == 2 )
			//	bin = 1;
			//if( binX == 3 )
			//	bin = 2;
			//if( binX == 4 )
			//	bin = 3;
			//if( binX == 8 )
			//	bin = 4;
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
std::cout << "Camera::getStatus -> no _sock !" << std::endl;
		return;
	}

	size_t acquireStatus, readoutStatus, correctStatus, writingStatus, dezingerStatus;

	DEB_MEMBER_FUNCT();

	this->get_marccd_state();
	
	size_t mar_status = TASK_STATE(this->_marccd_state);

	acquireStatus = TASK_STATUS(this->_marccd_state, TASK_ACQUIRE); 
	readoutStatus = TASK_STATUS(this->_marccd_state, TASK_READ); 
	correctStatus = TASK_STATUS(this->_marccd_state, TASK_CORRECT); 
	writingStatus = TASK_STATUS(this->_marccd_state, TASK_WRITE); 
	dezingerStatus= TASK_STATUS(this->_marccd_state, TASK_DEZINGER);

	if (this->_marccd_state == 0)                     m_status = Camera::Ready;
	else if (this->_marccd_state == 7)                m_status = Camera::Fault;
	else if (this->_marccd_state == 8)                m_status = Camera::Ready;  /* This is really busy interpreting command but we don't have a status for that yet */
	else if (acquireStatus & (TASK_STATUS_EXECUTING)) m_status = Camera::Exposure;
	else if (readoutStatus & (TASK_STATUS_EXECUTING)) m_status = Camera::Readout;
	else if (correctStatus & (TASK_STATUS_EXECUTING)) m_status = Camera::Readout;/*ADStatusCorrect*/
	else if (writingStatus & (TASK_STATUS_EXECUTING)) m_status = Camera::Readout;/*ADStatusSaving*/
	if ((acquireStatus | readoutStatus | correctStatus | writingStatus | dezingerStatus) & 	TASK_STATUS_ERROR)
		m_status = Camera::Fault;

	status = m_status;

	//DEB_TRACE() << "Camera::getStatus() - status = " << status;
	DEB_RETURN() << DEB_VAR1(DEB_HEX(status));
}

//-----------------------------------------------------
// - INTERNAL METHODS
//-----------------------------------------------------
void Camera::disconnect()
{
	try
	{
		this->_sock->disconnect();
		std::cout << "INFO::Camera::disconnect(): DONE !" << std::endl;
	}
	catch(yat::SocketException & ySe)
	{
		std::cout << "ERROR::Camera::disconnect -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
	}
	catch(...)
	{
		std::cout << "ERROR::Camera::disconnect -> [...] Exception." << std::endl;
	}
	//- TODO : gestion state de la com
	//this->com_state = Tango::CLOSE;
}

void Camera::connect()
{
	try
	{
		//this->_sock->set_option(yat::Socket::SOCK_OPT_KEEP_ALIVE, 1);
		this->_sock->set_option(yat::Socket::SOCK_OPT_NO_DELAY, 1);
		//this->_sock->set_option(yat::Socket::SOCK_OPT_OTIMEOUT, 0);
		//this->_sock->set_option(yat::Socket::SOCK_OPT_ITIMEOUT, 0);

		yat::Address addr(this->_camera_ip, this->_port_num);
		std::cout << "INFO::Camera::connect(): made Address... !" << std::endl;

		this->_sock->connect(addr);
		std::cout << "INFO::Camera::connect(): connect DONE." << std::endl;
	}
	//- TODO : on error -> sock = NULL
	catch(yat::SocketException & ySe)
	{
		this->_sock = 0;
		std::cout << "ERROR::Camera::connect -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
	}
	catch(...)
	{
		this->_sock = 0;
		std::cout << "ERROR::Camera::connect -> [...] Exception." << std::endl;
	}
}

//-----------------------------------------------------
// - read : returns Marccd command response
//-----------------------------------------------------
std::string Camera::read()
{
	std::string response("");
	try
	{
	//- TODO : CHECK if sock = NULL
		this->_sock->receive(response);
		//std::cout << "INFO::Camera::read(): response = " << response << std::endl;
	}
	catch(yat::SocketException & ySe)
	{
		std::cout << "ERROR::Camera::read -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
	}
	catch(...)
	{
		std::cout << "ERROR::Camera::read -> [...] Exception." << std::endl;
	}

	return response;
}

//-----------------------------------------------------
// - write : sends Marccd command
//-----------------------------------------------------
void Camera::write(std::string cmd_to_send)
{
	try
	{
		size_t data_size = cmd_to_send.size() + 1; //- +1 for '\0' character
	//- TODO : CHECK if sock = NULL
		this->_sock->send(cmd_to_send.c_str(), data_size);

//std::cout << "\n\nINFO::Camera::write(): cmd_to_be_send = " << cmd_to_send << " WRITTEN." << std::endl;
	}
	catch(yat::SocketException & ySe)
	{
		std::cout << "ERROR::Camera::write -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
	}
	catch(...)
	{
		std::cout << "ERROR::Camera::write -> [...] Exception." << std::endl;
	}
}

//-----------------------------------------------------
// - write_read : Sends a command and reads response
//-----------------------------------------------------
std::string Camera::write_read(std::string cmd_to_send)
{
	std::string response("");
	//- send command
	this->write(cmd_to_send);
	//- check if command needs a response
	if ( cmd_to_send.find("get_") != std::string::npos )
		response = this->read();

	return response;
}

//-----------------------------------------------------
// - perform_start_sequence : manage a Marccd acquisition
//-----------------------------------------------------
void Camera::perform_start_sequence()
{
std::cout << "Camera::perform_start_sequence <- " << std::endl;
	//---------------------------------------
	//-		PREPARE SEQUENCE
	//---------------------------------------
	//- Wait for Marccd to not be acquiring !
	do
	{
		this->get_marccd_state();
		std::cout << "Camera::perform_start_sequence -> ACQ on the way, state = " << this->_marccd_state << std::endl;
	}while(TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_EXECUTING));
std::cout << "\t **** Wait for Marccd to not be acquiring DONE." << std::endl;

	//---------------------------------------
	//-		TELL MARCCD START ACQUIRING
	//---------------------------------------
	//- Send start cmd
	std::string cmd_to_send("start");
{
	yat::MutexLock scoped_lock(this->_lock);        
	this->write_read(cmd_to_send);
}
std::cout << "\t **** START SENT !!!" << std::endl;

	//- Wait for Marccd to start acquiring : WHY not working => infinite loop !!!
	//do
	//{
	//	//yat::MutexLock scoped_lock(this->_lock);        
	//	this->get_marccd_state();
	//}while( !TEST_TASK_STATUS(this->_marccd_state, TASK_ACQUIRE, TASK_STATUS_EXECUTING) );

	//---------------------------------------
	//-		SHUTTER CONTROLLED IN DS -> TODO : to be implemented !!!
	//---------------------------------------
std::cout << "Camera::perform_start_sequence -> " << std::endl;
}

//-----------------------------------------------------
// - perform_stop_sequence : manage a Marccd END acquisition
//-----------------------------------------------------
void Camera::perform_stop_sequence()
{
std::cout << "Camera::perform_stop_sequence <- " << std::endl;

	//---------------------------------------
	//-		END ACQ : by starting readout -> TODO : to be implemented in STOP cmd !!!
	//---------------------------------------
	//- Send readout cmd with a specific file name
	std::string cmd_to_send("readout,0,");
	cmd_to_send += this->_image_path;
	cmd_to_send += this->_image_name;
	cmd_to_send += "_";
	cmd_to_send += yat::XString<size_t>::to_string(this->_image_number);
std::cout << "Camera::written file : &" << cmd_to_send << "$" << std::endl;

{
	yat::MutexLock scoped_lock(this->_lock);        
	this->write_read(cmd_to_send);
}
std::cout << "Camera::perform_stop_sequence -> " << std::endl;

	do
	{
		this->get_marccd_state();
		std::cout << "Camera::perform_stop_sequence -> CHECKING MARCCD STATE : " << this->_marccd_state << std::endl;
	}while(TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATE_WRITING));

	this->_image_number++;
}

//-----------------------------------------------------
// - perform_background_frame : sequence to take a background (dark) frame
//-----------------------------------------------------
void Camera::perform_background_frame()
{
std::cout << "\n\n\nCamera::perform_background_frame <- " << std::endl;

	//- wait for detector NOT be reading
	do
	{
		this->get_marccd_state();
		std::cout << "Camera::perform_background_frame -> wait for detector NOT be reading, state = " << this->_marccd_state << std::endl;
	}while(TEST_TASK_STATUS(this->_marccd_state,TASK_READ,TASK_STATUS_EXECUTING));
std::cout << "\t **** Wait for Marccd to not be acquiring DONE." << std::endl;

	//- Send readout 1 => read data into background frame storage
	std::string cmd_to_send("readout,1");
	{
		yat::MutexLock scoped_lock(this->_lock);        
		this->write_read(cmd_to_send);
	}
	
	this->get_marccd_state();
	while ( this->_marccd_state )
	{
		this->get_marccd_state();
	}
	///* Wait for the readout to start */
	//	this->get_marccd_state();
	//while (!TEST_TASK_STATUS(this->_marccd_state, TASK_READ, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED)) {
	//	this->get_marccd_state();
	//	//std::cout << "WAIT 4 R1 2 START : " << this->_marccd_state;
	//}
std::cout << "\n\t **** READOUT 1 DONE." << std::endl;
    /* Wait for the correction complete */
	//do
	//{
	//	this->get_marccd_state();
	//	//std::cout << "Camera::perform_background_frame -> readout_1 STATE : " << this->_marccd_state << std::endl;
	//}while(TEST_TASK_STATUS(this->_marccd_state, TASK_READ, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED));
	//std::cout << "Camera::perform_background_frame -> readout_1 FINISHED : state = " << this->_marccd_state << std::endl;



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
	}
std::cout << "\n\t **** READOUT 2 DONE." << std::endl;

	//- Send dezinger 1 => use and store into the current background frame
	cmd_to_send = "dezinger,1";
	{
		yat::MutexLock scoped_lock(this->_lock);        
		this->write_read(cmd_to_send);
	}
	//this->get_marccd_state();
	do
	{
		this->get_marccd_state();
		std::cout << "Camera::perform_background_frame -> dezinger STATE : " << this->_marccd_state << std::endl;
	}while(TEST_TASK_STATUS(this->_marccd_state,TASK_DEZINGER,TASK_STATUS_EXECUTING));
	//std::cout << "Camera::perform_background_frame -> readout_2 FINISHED : state = " << this->_marccd_state << std::endl;
	//std::cout << "Camera::perform_background_frame -> dezinger,1, state = " << this->_marccd_state << std::endl;

	
std::cout << "Camera::perform_background_frame -> \n\n" << std::endl;

	//do
	//{
	//	this->get_marccd_state();
	//	std::cout << "Camera::perform_background_frame -> CHECKING MARCCD STATE : " << this->_marccd_state << std::endl;
	//}while(TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATE_WRITING));
}

//-----------------------------------------------------
void Camera::get_marccd_state()
{
	std::string stateStr("");
	//- get detector state string value
	{
		yat::MutexLock scoped_lock(this->_lock);        
		stateStr = this->write_read("get_state");
	}
	
	//- convert state string to numeric val
	char data_to_conv[stateStr.size()+1];

	::strncpy(data_to_conv, stateStr.c_str(), stateStr.size());

	//this->_marccd_state  = yat::XString<size_t>::to_num(stateStr);
	this->_marccd_state  = this->convertStringToInt(data_to_conv);
}

//-----------------------------------------------------
// - getDirectoryWatcherPath
//-----------------------------------------------------
const std::string& Camera::getDirectoryWatcherPath(void)
{
  return _image_dir_watcher_path;
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

