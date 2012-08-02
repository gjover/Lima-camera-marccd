namespace Marccd
{

//*******************************************************************
// * \class Interface
// * \brief Marccd hardware interface
//*******************************************************************/
class Interface : HwInterface
{
%TypeHeaderCode
#include <MarccdInterface.h>
%End

 public:
	Interface(Marccd::Camera& cam);
	virtual ~Interface();

	//- From HwInterface
	//virtual void 	getCapList(CapList&) const;
	virtual void getCapList(std::vector<HwCap> &cap_list /Out/) const;	
	virtual void reset(ResetLevel reset_level);
	virtual void  prepareAcq();
	virtual void  startAcq();
	virtual void  stopAcq();
	virtual void  getStatus(StatusType& status);
	virtual int   getNbAcquiredFrames();
	virtual int   getNbHwAcquiredFrames();

	void  takeBackgroundFrame();
	void  getFrameRate(double& frame_rate);
	
	void setImageIndex(int imgIdx);
	int  getImageIndex();		
	
	void setImageFileName(const std::string& name);
	const std::string&  getImageFileName();
	
	void setImagePath(const std::string& path);
	const std::string&  getImagePath();
	
	unsigned int getCamState();

	//- Reader task timeout to process image from file
	//void setTimeout(int TO);
	
	//- get MARCCD image from file
        //void enableReader();
	//- get a simulated image
	//void disableReader();
	
};

}; // namespace Marccd











