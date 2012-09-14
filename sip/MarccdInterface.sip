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

	void takeBackgroundFrame();
	void saveBG();
	void getFrameRate(double& frame_rate);
	
	void setImageIndex(int imgIdx);
	int  getImageIndex();		
	
	void setImageFileName(const std::string& name);
	const std::string&  getImageFileName();
	
	void setImagePath(const std::string& path);
	const std::string&  getImagePath();
	
	unsigned int getCamState();

        void setBeamX(float);
        void setBeamY(float);
        void setDistance(float);
        void setWavelength(float);
        float getBeamX();
        float getBeamY();
        float getDistance();
        float getWavelength();

	SIP_PYDICT getHeader();	
%MethodCode
	int* hData;	
	PyObject* hDict = PyDict_New();

	hData = sipCpp->Interface::getHeader();
	PyDict_SetItemString(hDict, "distance", PyInt_FromLong(hData[160]));
	PyDict_SetItemString(hDict, "beam_x", PyInt_FromLong(hData[161]));
	PyDict_SetItemString(hDict, "beam_y", PyInt_FromLong(hData[162]));
	PyDict_SetItemString(hDict, "integration_time", PyInt_FromLong(hData[163]));
	PyDict_SetItemString(hDict, "exposure_time", PyInt_FromLong(hData[164]));
	PyDict_SetItemString(hDict, "readout_time", PyInt_FromLong(hData[165]));
	PyDict_SetItemString(hDict, "source_wavelength", PyInt_FromLong(hData[227]));
	PyDict_SetItemString(hDict, "optics_wavelength", PyInt_FromLong(hData[241]));
	PyDict_SetItemString(hDict, "acquire_timestamp", PyString_FromString((char*)&(hData[336])));
	PyDict_SetItemString(hDict, "header_timestamp", PyString_FromString((char*)&(hData[344])));
	PyDict_SetItemString(hDict, "save_timestamp", PyString_FromString((char*)&(hData[352])));

	PyDict_SetItemString(hDict, "mean", PyInt_FromLong(hData[72]));
	PyDict_SetItemString(hDict, "rms", PyInt_FromLong(hData[73]));

	PyDict_SetItemString(hDict, "pixelsize_x", PyInt_FromLong(hData[193]));
	PyDict_SetItemString(hDict, "pixelsize_y", PyInt_FromLong(hData[194]));
	PyDict_SetItemString(hDict, "mean_bias", PyInt_FromLong(hData[195]));
	PyDict_SetItemString(hDict, "photons_per_100adu", PyInt_FromLong(hData[196]));

	PyDict_SetItemString(hDict, "temperature_0", PyInt_FromLong(hData[206]));
	PyDict_SetItemString(hDict, "temperature_1", PyInt_FromLong(hData[207]));
	PyDict_SetItemString(hDict, "temperature_2", PyInt_FromLong(hData[208]));
	PyDict_SetItemString(hDict, "temperature_3", PyInt_FromLong(hData[209]));
	PyDict_SetItemString(hDict, "temperature_4", PyInt_FromLong(hData[210]));
	PyDict_SetItemString(hDict, "temperature_5", PyInt_FromLong(hData[211]));
	PyDict_SetItemString(hDict, "temperature_6", PyInt_FromLong(hData[212]));
	PyDict_SetItemString(hDict, "temperature_7", PyInt_FromLong(hData[213]));
	PyDict_SetItemString(hDict, "temperature_8", PyInt_FromLong(hData[214]));
	PyDict_SetItemString(hDict, "pressure_0", PyInt_FromLong(hData[215]));
	PyDict_SetItemString(hDict, "pressure_1", PyInt_FromLong(hData[216]));
	PyDict_SetItemString(hDict, "pressure_2", PyInt_FromLong(hData[217]));
	PyDict_SetItemString(hDict, "pressure_3", PyInt_FromLong(hData[218]));
	PyDict_SetItemString(hDict, "pressure_4", PyInt_FromLong(hData[219]));
	PyDict_SetItemString(hDict, "pressure_5", PyInt_FromLong(hData[220]));
	PyDict_SetItemString(hDict, "pressure_6", PyInt_FromLong(hData[221]));
	PyDict_SetItemString(hDict, "pressure_7", PyInt_FromLong(hData[222]));
	PyDict_SetItemString(hDict, "pressure_8", PyInt_FromLong(hData[223]));

	sipRes = hDict;
%End	

	//- Reader task timeout to process image from file
	//void setTimeout(int TO);
	
	//- get MARCCD image from file
        //void enableReader();
	//- get a simulated image
	//void disableReader();
	
};

}; // namespace Marccd











