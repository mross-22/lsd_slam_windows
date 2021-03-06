#pragma once


#include <mfobjects.h>


#include "videoInput.h"
#include "VideoCaptureSession.h"
#include "VideoCaptureSink.h"
#include "BoxMode.h"
#include "IReadWriteBuffer.h"



struct IMFMediaSource;


class VideoCaptureDevice: public IUnknown
{
public:
	VideoCaptureDevice(void);
	virtual ~VideoCaptureDevice(void);

	ResultCode::Result init(IMFMediaSource *pSource, DeviceSettings deviceSettings, CaptureSettings captureSettings, MediaType MT);

	ResultCode::Result startDevice();

	ResultCode::Result closeDevice();

	ResultCode::Result readPixels(unsigned char *pPixels);

	ResultCode::Result getParametrs(CamParametrsSetting &parametrs);

	ResultCode::Result setParametrs(CamParametrsSetting parametrs);
	
    STDMETHODIMP QueryInterface(REFIID iid, void** ppv);

	STDMETHODIMP_(ULONG) AddRef();
        
	STDMETHODIMP_(ULONG) Release();

CAPTUREMANAGER_BOX

	ResultCode::Result createTopology(IMFMediaSource *pSource, DWORD streamIndex , IMFActivate *pSinkActivate, IMFTopology *pTopology);

	ResultCode::Result addSourceNode(IMFTopology *pTopology, IMFMediaSource *pSource, IMFPresentationDescriptor *pPD, IMFStreamDescriptor *pSD, IMFTopologyNode **ppNode);

	ResultCode::Result addSinkNode(IMFTopology *pTopology, IMFActivate *pActivate, DWORD dwId, IMFTopologyNode **ppNode);
	
	


	volatile long refCount;  

	CamParametrs prevParametrs;

	CComAutoCriticalSection critSec;

	CComPtr<IMFMediaSource> pLocalSource;

	CComPtr<VideoCaptureSession> pLocalSession;

	CComPtr<VideoCaptureSink> pLocalSink;
	
	CComPtr<IMFMediaSession> pSession;

	CComPtr<IReadWriteBuffer> pIReadWriteBuffer;


};


