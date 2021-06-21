#include "Accel.h"
#include <sstream>
#include "researchmode/ResearchModeApi.h"
#include <fstream>
#include <iomanip>
#include <ctime>
#include <time.h>


using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Storage;


//this is the thread in which we constantly
//give permission to continue accessing
//the sensors
void Accel::AccelUpdateThread(Accel* pAccelReader, HANDLE imuConsentGiven, ResearchModeSensorConsent* imuAccessConsent)
{
    HRESULT hr = S_OK;

    //open up the stream so that we can extract
    //sensor data from it

    //if succeeded
    if (SUCCEEDED(hr))
    {
        hr = pAccelReader->m_pAccelSensor->OpenStream();
        if (FAILED(hr))
        {
            pAccelReader->m_pAccelSensor->Release();
            pAccelReader->m_pAccelSensor = nullptr;
        }

        while (!pAccelReader->m_fExit && pAccelReader->m_pAccelSensor)
        {
            //assign fundamentals
            HRESULT hr = S_OK;
            IResearchModeSensorFrame* pSensorFrame = nullptr;


            hr = pAccelReader->m_pAccelSensor->GetNextBuffer(&pSensorFrame);
            if (SUCCEEDED(hr))
            {
                std::lock_guard<std::mutex> guard(pAccelReader->m_sensorFrameMutex);
                
                if (pAccelReader->m_pSensorFrame)
                {
                    pAccelReader->m_pSensorFrame->Release();
                }
                pAccelReader->m_pSensorFrame = pSensorFrame;
            }

        }

        if (pAccelReader->m_pAccelSensor)
        {
            pAccelReader->m_pAccelSensor->CloseStream();
        }

    }

}

bool Accel::IsNewTimestamp(IResearchModeSensorFrame* pSensorFrame)
{
    ResearchModeSensorTimestamp timeStamp;
    winrt::check_hresult(pSensorFrame->GetTimeStamp(&timeStamp));

    if (m_prevTimestamp == timeStamp.HostTicks)
    {
        return false;
    }

    m_prevTimestamp = timeStamp.HostTicks;

    return true;
}


void Accel::AccelWriteThread(Accel* pReader)
{
    while (!pReader->m_fExit)
    {

        std::unique_lock<std::mutex> storage_lock(pReader->m_storageMutex);
        if (pReader->m_storageFolder == nullptr)
        {
            pReader->m_storageCondVar.wait(storage_lock);
        }

        std::lock_guard<std::mutex> reader_guard(pReader->m_sensorFrameMutex);
        if (pReader->m_pSensorFrame)
        {
            if (pReader->IsNewTimestamp(pReader->m_pSensorFrame))
            {
                pReader->SaveFrame(pReader->m_pSensorFrame);

            }
        }
    }
}

void Accel::SaveFrame(IResearchModeSensorFrame* pSensorFrame)
{

    IResearchModeAccelFrame* pSensorAccelFrame = nullptr;
    HRESULT   hr = pSensorFrame->QueryInterface(IID_PPV_ARGS(&pSensorAccelFrame));

    if (pSensorAccelFrame)
    {
        SaveFiles(pSensorFrame, pSensorAccelFrame);
        pSensorAccelFrame->Release();
    }
}
//this is triggered when the write thread above is triggered
void Accel::SaveFiles(IResearchModeSensorFrame* pSensorFrame, IResearchModeAccelFrame* pSensorAccelFrame)
{
    DirectX::XMFLOAT3 sample;
    char printString[1000];
    HRESULT hr = S_OK;

    ResearchModeSensorTimestamp timeStamp;
    UINT64 lastSocTickDelta = 0;
    UINT64 glastSocTick = 1;

    pSensorFrame->GetTimeStamp(&timeStamp);

    if (glastSocTick != 0)
    {
        lastSocTickDelta = timeStamp.HostTicks - glastSocTick;
    }
    glastSocTick = timeStamp.HostTicks;

    hr = pSensorAccelFrame->GetCalibratedAccelaration(&sample);
    if (FAILED(hr))
    {
        return;
    }

    //time related data is created here
    //float                     gettime                        = m_timer.GetTime();
    //static unsigned long long getsystemrelativetime          = m_timer.GetSystemRelativeTime();
    //static double             getsystemrelativetimeinseconds = m_timer.GetSystemRelativeTimeInSeconds();
    //static unsigned long long getfiletime                    = m_timer.GetFileTime();



    //timerconverter related data
    HundredsOfNanoseconds timestamp             = m_converter.RelativeTicksToAbsoluteTicks(HundredsOfNanoseconds((long long)m_prevTimestamp));
    HundredsOfNanoseconds universalstamp        = m_converter.UniversalToUnixTime(fileTime);
    //HundredsOfNanoseconds filetimetoabsticks    = m_converter.FileTimeToAbsoluteTicks(fileTime);
    //HundredsOfNanoseconds reltoabsticksoffset   = m_converter.CalculateRelativeToAbsoluteTicksOffset();

    



    sprintf_s(printString, "####Accel: % 3.6f % 3.6f % 3.6f %f %I64d %I64d %I64d\n",
        sample.x,
        sample.y,
        sample.z,
        sqrt(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z),
        (lastSocTickDelta * 1000) / timeStamp.HostTicksPerSecond,
        timestamp.count(),
        universalstamp.count(),
    );

    size_t outIMUBufferCount = 0;
    wchar_t outputIMUPath[MAX_PATH];


    std::string printStringAsStdStr = printString;
    swprintf_s(outputIMUPath, L"%llu_imu.txt", timestamp.count());
    std::vector<BYTE> IMUData;
    IMUData.reserve(printStringAsStdStr.size() + outIMUBufferCount * sizeof(UINT16));
    IMUData.insert(IMUData.end(), printStringAsStdStr.c_str(), printStringAsStdStr.c_str() + printStringAsStdStr.size());
    m_tarball->AddFile(outputIMUPath, &IMUData[0], IMUData.size());


    return;
}

//sets its own tar ball so that info can be recorded inside
void Accel::AccelSetStorageFolder(const winrt::Windows::Storage::StorageFolder& storageFolder)
{
    std::lock_guard<std::mutex> storage_guard(m_storageMutex);
    m_storageFolder = storageFolder;
    wchar_t fileName[MAX_PATH] = {};
    swprintf_s(fileName, L"%s\\%s.tar", m_storageFolder.Path().data(), m_pAccelSensor->GetFriendlyName());
    m_tarball.reset(new Io::Tarball(fileName));
    m_storageCondVar.notify_all();

}


//dumps IMU extrinsics as the streams close down and the recording is finished
void Accel::AccelResetStorageFolder()
{
    std::lock_guard<std::mutex> storage_guard(m_storageMutex);
    DumpFiles();

}


void Accel::DumpFiles()  //dumps extrinsics at the end of the stream
{
    {
        std::lock_guard<std::mutex> guard(m_sensorFrameMutex);
        // Assuming we are at the end of the capture
        assert(m_pSensorFrame != nullptr);
    }


    // Get accel sensor object
    IResearchModeAccelSensor* pAccelSensor = nullptr;
    HRESULT hr = m_pAccelSensor->QueryInterface(IID_PPV_ARGS(&pAccelSensor));
    winrt::check_hresult(hr);

    wchar_t outputExtrinsicsPath[MAX_PATH] = {};
    swprintf_s(outputExtrinsicsPath, L"%s\\%s_acceldata_extrinsics.txt", m_storageFolder.Path().data(), m_pAccelSensor->GetFriendlyName());

    std::ofstream fileExtrinsics(outputExtrinsicsPath);
    DirectX::XMFLOAT4X4 accel_extrinsics;

    pAccelSensor->GetExtrinsicsMatrix(&accel_extrinsics);


    fileExtrinsics << accel_extrinsics.m[0][0] << "," << accel_extrinsics.m[1][0] << "," << accel_extrinsics.m[2][0] << "," << accel_extrinsics.m[3][0] << ","
        << accel_extrinsics.m[0][1] << "," << accel_extrinsics.m[1][1] << "," << accel_extrinsics.m[2][1] << "," << accel_extrinsics.m[3][1] << ","
        << accel_extrinsics.m[0][2] << "," << accel_extrinsics.m[1][2] << "," << accel_extrinsics.m[2][2] << "," << accel_extrinsics.m[3][2] << ","
        << accel_extrinsics.m[0][3] << "," << accel_extrinsics.m[1][3] << "," << accel_extrinsics.m[2][3] << "," << accel_extrinsics.m[3][3] << "\n";

    fileExtrinsics.close();
    pAccelSensor->Release();


}
