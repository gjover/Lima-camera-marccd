#include "MarccdInterface.h"
#include <algorithm>

using namespace lima;
using namespace lima::Marccd;
using namespace std;

#define MINBINX 2
#define MAXBINX 8
#define MINBINY 2
#define MAXBINY 8

/*******************************************************************
 * \brief DetInfoCtrlObj constructor
 *******************************************************************/
DetInfoCtrlObj::DetInfoCtrlObj(Camera& cam)   :m_cam(cam)
{
  DEB_CONSTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
DetInfoCtrlObj::~DetInfoCtrlObj()
{
  DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getMaxImageSize(Size& size)
{
  DEB_MEMBER_FUNCT();
  m_cam.getImageSize(size);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getDetectorImageSize(Size& size)
{
  DEB_MEMBER_FUNCT();
  m_cam.getImageSize(size);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getDefImageType(ImageType& image_type)
{
  DEB_MEMBER_FUNCT();
	m_cam.getImageType(image_type);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getCurrImageType(ImageType& image_type)
{
  DEB_MEMBER_FUNCT();
	m_cam.getImageType(image_type);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::setCurrImageType(ImageType image_type)
{
  DEB_MEMBER_FUNCT();
  ImageType valid_image_type;
  getDefImageType(valid_image_type);
  if (image_type != valid_image_type)
  THROW_HW_ERROR(Error) << "Cannot change to "  << DEB_VAR2(image_type, valid_image_type);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getPixelSize(double& x_size,double& y_size)
{
  DEB_MEMBER_FUNCT();
  m_cam.getPixelSize(x_size,y_size);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getDetectorType(std::string& type)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorType(type);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getDetectorModel(std::string& model)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorModel(model);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
{
  DEB_MEMBER_FUNCT();
	m_mis_cb_gen.registerMaxImageSizeCallback(cb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
{
  DEB_MEMBER_FUNCT();
	m_mis_cb_gen.unregisterMaxImageSizeCallback(cb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::MaxImageSizeCallbackGen::setMaxImageSizeCallbackActive(bool )
{
}

/*******************************************************************
 * \brief BufferCtrlObj constructor
 *******************************************************************/

BufferCtrlObj::BufferCtrlObj(Camera& cam)
	:   m_buffer_cb_mgr(m_buffer_alloc_mgr),
      m_buffer_ctrl_mgr(m_buffer_cb_mgr),
      m_cam(cam)
{
	DEB_CONSTRUCTOR();
  m_reader = new Reader(cam,*this);
  m_reader->go(2000);  
}

//-----------------------------------------------------
//
//-----------------------------------------------------
BufferCtrlObj::~BufferCtrlObj()
{
	DEB_DESTRUCTOR();
  m_reader->reset();
  m_reader->exit();  
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::setFrameDim(const FrameDim& frame_dim)
{
	DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.setFrameDim(frame_dim);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getFrameDim(FrameDim& frame_dim)
{
	DEB_MEMBER_FUNCT();
	m_buffer_ctrl_mgr.getFrameDim(frame_dim);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::setNbBuffers(int nb_buffers)
{
	DEB_MEMBER_FUNCT();
	m_buffer_ctrl_mgr.setNbBuffers(nb_buffers);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getNbBuffers(int& nb_buffers)
{
	DEB_MEMBER_FUNCT();
	m_buffer_ctrl_mgr.getNbBuffers(nb_buffers);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::setNbConcatFrames(int nb_concat_frames)
{
	DEB_MEMBER_FUNCT();
	m_buffer_ctrl_mgr.setNbConcatFrames(nb_concat_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getNbConcatFrames(int& nb_concat_frames)
{
	DEB_MEMBER_FUNCT();
	m_buffer_ctrl_mgr.getNbConcatFrames(nb_concat_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getMaxNbBuffers(int& max_nb_buffers)
{
	DEB_MEMBER_FUNCT();
  m_buffer_ctrl_mgr.getMaxNbBuffers(max_nb_buffers);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void *BufferCtrlObj::getBufferPtr(int buffer_nb, int concat_frame_nb)
{
	DEB_MEMBER_FUNCT();
  return m_buffer_ctrl_mgr.getBufferPtr(buffer_nb, concat_frame_nb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void *BufferCtrlObj::getFramePtr(int acq_frame_nb)
{
	DEB_MEMBER_FUNCT();
  return m_buffer_ctrl_mgr.getFramePtr(acq_frame_nb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getStartTimestamp(Timestamp& start_ts)
{
	DEB_MEMBER_FUNCT();
	m_buffer_ctrl_mgr.getStartTimestamp(start_ts);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getFrameInfo(int acq_frame_nb, HwFrameInfoType& info)
{
	DEB_MEMBER_FUNCT();
	m_buffer_ctrl_mgr.getFrameInfo(acq_frame_nb, info);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::registerFrameCallback(HwFrameCallback& frame_cb)
{
	DEB_MEMBER_FUNCT();
	m_buffer_ctrl_mgr.registerFrameCallback(frame_cb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::unregisterFrameCallback(HwFrameCallback& frame_cb)
{
	DEB_MEMBER_FUNCT();
	m_buffer_ctrl_mgr.unregisterFrameCallback(frame_cb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::update_image_from_file()
{
    DEB_MEMBER_FUNCT();
		//- start thread which get the new image from file
    m_reader->start();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::reset()
{
  DEB_MEMBER_FUNCT();
  m_reader->reset();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int BufferCtrlObj::getLastAcquiredFrame()
{
  return m_reader->getLastAcquiredFrame();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool BufferCtrlObj::isTimeoutSignaled()
{
    return m_reader->isTimeoutSignaled();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool BufferCtrlObj::isRunning()
{
    return m_reader->isRunning();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::setTimeout(int TO)
{
    DEB_MEMBER_FUNCT();
    m_reader->setTimeout(TO);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::enableReader(void)
{
    DEB_MEMBER_FUNCT();
	m_reader->enableReader();
std::cout << "\t\tBufferCtrlObj::enableReader(void) -> DONE" << std::endl;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::disableReader(void)
{
    DEB_MEMBER_FUNCT();
	m_reader->disableReader();
std::cout << "\t\tBufferCtrlObj::disableReader(void) -> DONE" << std::endl;
}

int* BufferCtrlObj::getHeader()
{
  DEB_MEMBER_FUNCT();
  return m_reader->getHeader();
}

/*******************************************************************
 * \brief SyncCtrlObj constructor
 *******************************************************************/

SyncCtrlObj::SyncCtrlObj(Camera& cam)
            : m_cam(cam)
{
}

//-----------------------------------------------------
//
//-----------------------------------------------------
SyncCtrlObj::~SyncCtrlObj()
{
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool SyncCtrlObj::checkTrigMode(TrigMode trig_mode)
{
	bool valid_mode = false;
	switch (trig_mode)
	{
		case IntTrig:
			valid_mode = true;
		break;

		default:
			valid_mode = false;
	}
	return valid_mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();    
	if (!checkTrigMode(trig_mode))
		THROW_HW_ERROR(InvalidValue) << "Invalid " << DEB_VAR1(trig_mode);
	m_cam.setTrigMode(trig_mode);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getTrigMode(TrigMode& trig_mode)
{
	m_cam.getTrigMode(trig_mode);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setExpTime(double exp_time)
{
	m_cam.setExpTime(exp_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getExpTime(double& exp_time)
{
	m_cam.getExpTime(exp_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setLatTime(double lat_time)
{
	m_cam.setLatTime(lat_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getLatTime(double& lat_time)
{
	m_cam.getLatTime(lat_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setNbFrames(int nb_frames)
{
	m_cam.setNbFrames(nb_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getNbFrames(int& nb_frames)
{
	m_cam.getNbFrames(nb_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setNbHwFrames(int nb_frames)
{
	m_cam.setNbFrames(nb_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getNbHwFrames(int& nb_frames)
{
	m_cam.getNbFrames(nb_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getValidRanges(ValidRangesType& valid_ranges)
{
	double min_time = 10e-9;
	double max_time = 1e6;
	valid_ranges.min_exp_time = min_time;
	valid_ranges.max_exp_time = max_time;
	valid_ranges.min_lat_time = min_time;
	valid_ranges.max_lat_time = max_time;
}

/*******************************************************************
 * \brief RoiCtrlObj constructor
 *******************************************************************/
RoiCtrlObj::RoiCtrlObj(Camera& cam)
    : m_cam(cam)
{
    DEB_CONSTRUCTOR();
    
}

//-----------------------------------------------------
//
//-----------------------------------------------------
RoiCtrlObj::~RoiCtrlObj()
{
    DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void RoiCtrlObj::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    m_cam.checkRoi(set_roi, hw_roi);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void RoiCtrlObj::setRoi(const Roi& roi)
{
    DEB_MEMBER_FUNCT();
    Roi real_roi;
    checkRoi(roi,real_roi);
    m_cam.setRoi(real_roi);

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void RoiCtrlObj::getRoi(Roi& roi)
{
    DEB_MEMBER_FUNCT();
    m_cam.getRoi(roi);
}

/*******************************************************************
 * \brief BinCtrlObj constructor
 *******************************************************************/
BinCtrlObj::BinCtrlObj(Camera &cam) 
: m_cam(cam) 
{
	//- Noop
}

/*******************************************************************
 * \brief setBin : set new binning value
 *******************************************************************/
void BinCtrlObj::setBin(const Bin& aBin)
{
  Bin myBin = aBin;
//   std::cout   << "BinCtrlObj::setBin " 
// 	      << aBin.getX() << "x"
// 	      << aBin.getY() << std::endl;  
  checkBin(myBin);
//   std::cout   << "---> After check:  " 
// 	      << myBin.getX() << "x"
// 	      << myBin.getY() << std::endl;  
  m_cam.setBinning(myBin);
}

/*******************************************************************
 * \brief getBin : returns binning value
 *******************************************************************/
void BinCtrlObj::getBin(Bin &aBin)
{
  m_cam.getBinning(aBin);
}

/*******************************************************************
 * \brief checkBin : returns a validated binning value
 *******************************************************************/
void BinCtrlObj::checkBin(Bin& bin)
{
  int bin_x = min(bin.getX(), int(MAXBINX));
  bin_x = max(bin_x, int(MINBINX));
  int bin_y = min(bin.getY(), int(MAXBINY));
  bin_y = max(bin_y, int(MINBINY));
  bin = Bin(bin_x, bin_y);
}

/*******************************************************************
 * \brief Hw Interface constructor
 *******************************************************************/

Interface::Interface(Camera& cam)
	: m_cam(cam),m_det_info(cam), m_buffer(cam),m_sync(cam),m_roi(cam), m_bin(cam)
{
	DEB_CONSTRUCTOR();

  //std::cout   << "Interface::Interface - ENTERING" << std::endl;
  
  //run the cammera
  m_cam.go(2000);

	HwDetInfoCtrlObj *det_info = &m_det_info;
	m_cap_list.push_back(HwCap(det_info));

	HwBufferCtrlObj *buffer = &m_buffer;
	m_cap_list.push_back(HwCap(buffer));
	
	HwSyncCtrlObj *sync = &m_sync;
	m_cap_list.push_back(HwCap(sync));
			
	HwRoiCtrlObj *roi = &m_roi;
	m_cap_list.push_back(HwCap(roi));

	HwBinCtrlObj *bin = &m_bin;
	m_cap_list.push_back(HwCap(bin));
  
  m_cam.stop();
  m_buffer.reset();

  //std::cout   << "Interface::Interface() - DONE" << std::endl;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Interface::~Interface()
{
	DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getCapList(HwInterface::CapList &cap_list) const
{
	DEB_MEMBER_FUNCT();
	cap_list = m_cap_list;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::reset(ResetLevel reset_level)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(reset_level);

  //stopAcq();

  /*
	Size image_size;
	m_det_info.getMaxImageSize(image_size);
	ImageType image_type;
	m_det_info.getDefImageType(image_type);
	FrameDim frame_dim(image_size, image_type);
	m_buffer.setFrameDim(frame_dim);

	m_buffer.setNbConcatFrames(1);
	m_buffer.setNbBuffers(1);
  */
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::prepareAcq()
{
	DEB_MEMBER_FUNCT();
  m_buffer.reset();
	m_cam.prepare();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::startAcq()
{
	DEB_MEMBER_FUNCT();
	m_cam.start();
  m_buffer.update_image_from_file(); 
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::stopAcq()
{
	DEB_MEMBER_FUNCT();
	m_cam.stop();
  m_buffer.reset();
  //m_buffer.update_image_from_file();    
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::takeBackgroundFrame()
{
	DEB_MEMBER_FUNCT();
	m_cam.take_background_frame();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::saveBG()
{
  DEB_MEMBER_FUNCT();
  m_cam.saveBG(true);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getStatus(StatusType& status)
{
	Camera::Status marccd_status = Camera::Unknown;
	

////std::cout << "\t***** Interface::getStatus -> MARCCD status = " << marccd_status << std::endl;

  int nb_frames;
  m_cam.getNbFrames(nb_frames);

	if( this->m_buffer.isRunning() )
	{
		status.acq = AcqRunning;
		status.det = DetExposure;
	}
  else if (m_buffer.isTimeoutSignaled())
    {
      status.acq = AcqFault;
      status.det = DetFault;
    }
	else
	{
		m_cam.getStatus(marccd_status);
		switch (marccd_status)
		{
		case Camera::Ready:
			status.acq = AcqReady;
			status.det = DetIdle;
			break;

		case Camera::Exposure:
			status.acq = AcqRunning;
			status.det = DetExposure;
			break;

		case Camera::Readout:
			status.acq = AcqRunning;
			status.det = DetReadout;
			break;

		case Camera::Latency:
			status.acq = AcqRunning;
			status.det = DetLatency;
			break;

	case Camera::Config:
	  status.acq = AcqConfig;
	  status.det = DetFault;
	  break;

		case Camera::Unknown:
		case Camera::Fault:
			status.acq = AcqFault;
			status.det = DetFault;
			break;
		}
	}
	status.det_mask = DetExposure | DetReadout | DetLatency | DetFault;
/////std::cout << "\t***** Interface::getStatus -> StatusType = " << status.acq << std::endl;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Interface::getNbAcquiredFrames()
{
  DEB_MEMBER_FUNCT();
	int acq_frames = m_buffer.getLastAcquiredFrame();
	return acq_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Interface::getNbHwAcquiredFrames()
{
	DEB_MEMBER_FUNCT();
	int acq_frames = m_buffer.getLastAcquiredFrame();
	return acq_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getFrameRate(double& frame_rate)
{
	DEB_MEMBER_FUNCT();
	m_cam.getFrameRate(frame_rate);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setImageFileName(const std::string& name)
{
	DEB_MEMBER_FUNCT();
	m_cam.setImageFileName(name);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
const std::string& Interface::getImageFileName(void)
{
	DEB_MEMBER_FUNCT();
	return m_cam.getImageFileName();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setImagePath(const std::string& path)
{
	DEB_MEMBER_FUNCT();
	m_cam.setImagePath(path);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
const std::string& Interface::getImagePath(void)
{
	DEB_MEMBER_FUNCT();
	return m_cam.getImagePath();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setImageIndex(int imgIdx)
{
	DEB_MEMBER_FUNCT();
	this->m_cam.setImageIndex(imgIdx);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Interface::getImageIndex()
{
	DEB_MEMBER_FUNCT();
	return m_cam.getImageIndex();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
unsigned int Interface::getCamState()
{
  DEB_MEMBER_FUNCT();
  return m_cam.getState();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setBeamX(float X)
{
  DEB_MEMBER_FUNCT();
  m_cam.setBeamX(X);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setBeamY(float Y)
{
  DEB_MEMBER_FUNCT();
  m_cam.setBeamY(Y);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setDistance(float D)
{
  DEB_MEMBER_FUNCT();
  m_cam.setDistance(D);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setWavelength(float W)
{
  DEB_MEMBER_FUNCT();
  m_cam.setWavelength(W);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
float Interface::getBeamX()
{
  DEB_MEMBER_FUNCT();
  return m_cam.getBeamX();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
float Interface::getBeamY()
{
  DEB_MEMBER_FUNCT();
  return m_cam.getBeamY();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
float Interface::getDistance()
{
  DEB_MEMBER_FUNCT();
  return m_cam.getDistance();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
float Interface::getWavelength()
{
  DEB_MEMBER_FUNCT();
  return m_cam.getWavelength();
}

//-----------------------------------------------------
// get last buffer header
//-----------------------------------------------------
int* Interface::getHeader(void)
{
  DEB_MEMBER_FUNCT();
  return m_buffer.getHeader();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setTimeout(int TO)
{
    DEB_MEMBER_FUNCT();
    m_buffer.setTimeout(TO);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::enableReader(void)
{
	m_buffer.enableReader();
std::cout << "\tInterface::enableReader(void) -> DONE" << std::endl;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::disableReader(void)
{
	m_buffer.disableReader();
std::cout << "\tInterface::disableReader(void) -> DONE" << std::endl;
}
