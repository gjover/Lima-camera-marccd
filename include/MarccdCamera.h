#ifndef _MARCCD_INTERFACE_H
#define _MARCCD_INTERFACE_H

///////////////////////////////////////////////////////////
// YAT::TASK 
///////////////////////////////////////////////////////////
#include <yat/threading/Task.h>
#include <yat/network/ClientSocket.h>
//#include <yat/network/Address.h>

#define kLO_WATER_MARK      128
#define kHI_WATER_MARK      512

#define kPOST_MSG_TMO       2

///////////////////////////////////////////////////////////

#include <stdlib.h>
#include <limits>

#include "HwMaxImageSizeCallback.h"
#include "HwBufferMgr.h"

namespace lima
{

	namespace Marccd
	{

		//--------------------------------------------------------------------
		// \class Camera
		// \brief object controlling the marccd detector through a socket
		//--------------------------------------------------------------------
		class Camera : public yat::Task
		{
			DEB_CLASS_NAMESPC(DebModCamera, "Camera", "MarCCD");

		public:

	enum Status 
	{
				Ready, 
				Exposure,
				Readout,
				Latency,
	  Config,
				Fault,
				Unknown
			};

			Camera(const std::string& camera_ip, 
						 size_t port_number, 
						 const std::string& img_path);

	//Camera();
	Camera(const Camera&);
	~Camera();
	Camera& operator=(const Camera&);

			void start();
			void stop();
			void take_background_frame();

			void prepare(); //- USEFULL ?

			// -- detector info
			void getImageSize(Size& size);
	//void setPixelDepth(ImageType pixel_depth);
	//void getPixelDepth(ImageType& pixel_depth);
			void getPixelSize(double& x_size,double &y_size);
			void getImageType(ImageType& type);

			void getDetectorType(std::string& type);
			void getDetectorModel(std::string& model);

			//- Sync 
			void setTrigMode(TrigMode  mode);
			void getTrigMode(TrigMode& mode);

			void setExpTime(double  exp_time);
			void getExpTime(double& exp_time);

			void setLatTime(double  lat_time);
			void getLatTime(double& lat_time);

			void setNbFrames(int  nb_frames);
			void getNbFrames(int& nb_frames);

			void getStatus(Camera::Status& status);
	unsigned int getState();
			void getFrameRate(double& frame_rate);

			void checkRoi(const Roi& set_roi, Roi& hw_roi);
			void setRoi(const Roi& set_roi);
			void getRoi(Roi& hw_roi);    

			void setBinning(const Bin&);
			void getBinning(Bin&);
			void checkBin(Bin& );
      
	bool is_stop_sequence_finished();

			void  setImagePath(const std::string& path);
			const std::string&  getImagePath(void);

	//const std::string& getDirectoryWatcherPath();

			void setImageFileName(const std::string& imgName);
			const std::string& getImageFileName();

      const std::string& getFullImgName();

	void setImageIndex(int newImgIdx);
	int getImageIndex() ;
	int getFirstImage() ;
	void saveBG(bool);

        void setBeamX(float);
        void setBeamY(float);
        void setDistance(float);
        void setWavelength(float);
	float getBeamX();
	float getBeamY();
	float getDistance();
	float getWavelength();

		protected:
			virtual void setMaxImageSizeCallbackActive(bool cb_active);	

			//- yat::Task implementation
			virtual void handle_message( yat::Message& msg )      throw (yat::Exception);
		private:
			//void GetImage();
			//void FreeImage();

			/**
			* Disconnect from the current socket
			*/
			void disconnect();
			/**
			* Connect to the current socket
			*/
			void connect();
			/**
			* Get detector response
			*/
			std::string read();
			/**
			* Send command to detector
			*/
			void write(std::string);
			/**
			* Send command and receive the response
			*/
			std::string write_read(std::string);
			/**
			* update marccd state
			*/
			void get_marccd_state();
#if 0
			/**
			* Method to start an Marccd acquisition
			*/
			void perform_start_sequence();
			/**
			* Method to stop an Marccd acquisition
			*/
			void perform_stop_sequence();
#endif
	/**
	 * Method to perform an Marccd acquisition
	 */
	void perform_acquisition_sequence();
	/**
	 * Method to abort an Marccd acquisition
	 */
	void perform_abort_sequence();
			/**
			* Sequence to take a background frame
			*/
			void perform_background_frame();

			int convertStringToInt( char* hexa_text );
			      
			//- mutex to protect file against read image from device and
			//-		marccd acquisition
			yat::Mutex 	        _lock;
			yat::ClientSocket*  _sock;
			
			std::string         _image_path;
			std::string         _image_name;
	std::string         _full_img_name;

			//- img stuff
			int 	m_nb_frames;	
			Size	m_image_size;
      
			unsigned short	    m_trigger_type;
	double         m_exp_time;
	double         m_lat_time;
	int            m_binning;
	
			size_t _marccd_state;
			Camera::Status		  m_status;
			int							    _image_number;
	int            _first_image;

			//- Marccd stuff 
			std::string		      _camera_ip;
			size_t				      _port_num;
			std::string 	      _detector_model;
			std::string 	      _detector_type;

	// X-ray source header info
	float sourceBeamX;
	float sourceBeamY;
	float sourceDistance;
	float sourceWavelength;

	//bool _stop_already_done;
			std::string					_error;
	bool _stop_sequence_finished;
	bool _abort;

	bool _bgAcquired;
	bool _saveBG;
		};
	} //- namespace Marccd
} // namespace lima


#endif // _MARCCD_INTERFACE_H
