#include <opencv2/opencv.hpp>

//* log with fprintf *//
#ifndef LOGPF
#define LOGPF(format, ...) fprintf(stderr ,"[%s:%d] " format "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#endif

int main(int argc, char **argv)
{
    for(int i = 0; i < argc; i++){
        LOGPF("argv[%d] = %s\n", i, argv[i]);
    }

    if(argc < 4)
    {
        LOGPF("missing args: test_raw12_bin [img_path] [w] [h] [format]");
        return 0;
    }

    /** input args */
    const std::string raw12_img_path(argv[1]);
    const int w = atoi(argv[2]);
    const int h = atoi(argv[3]);
    int cvt_code = 0;

    if(strncmp(argv[4], "bggr", 4) == 0)
    {
        cvt_code = cv::COLOR_BayerRG2BGR;
    }
    else if(strncmp(argv[4], "gbrg", 4) == 0)
    {
        cvt_code = cv::COLOR_BayerGR2BGR;
    }
    else if(strncmp(argv[4], "grbg", 4) == 0)
    {
        cvt_code = cv::COLOR_BayerGB2BGR;
    }
    else if(strncmp(argv[4], "rggb", 4) == 0)
    {
        cvt_code = cv::COLOR_BayerBG2BGR;
    }
    else
    {
        LOGPF("wrong bayer format!");
        return 0;
    }

    FILE * pFile;
    long byte_size;
    long pixel_size;
    char* byte_buf;
    uint16_t* img_buf;
    size_t result;

    pFile = fopen ( argv[1] , "rb" );
    if (pFile==NULL) 
    {
        LOGPF("fopen %s error", argv[1]);
        return -1;
    }

    // obtain file size:
    fseek (pFile , 0 , SEEK_END);
    byte_size = ftell (pFile);
    rewind (pFile);

    LOGPF("image size: %ld", byte_size);
    /** image format should be raw12 packed */
    pixel_size = h*w;
    assert(byte_size == long(pixel_size*1.5f));

    // allocate memory to contain the whole file:
    byte_buf = (char*) malloc (sizeof(char)*byte_size);
    img_buf = (uint16_t*) malloc (sizeof(uint16_t)*pixel_size);
    if(byte_buf == NULL) 
    {
        LOGPF("malloc byte_buf error");
        return -2;
    }

    // copy the file into the byte_buf:
    result = fread (byte_buf, 1, byte_size, pFile);
    if (result != byte_size) 
    {
        LOGPF("fread error");
        return -3;
    }

    /** raw12 packed to uint16, big-endian */
    for(long i=0; i<pixel_size-1; i+=2)
    {
        img_buf[i] = byte_buf[long(1.5f*i)] + (byte_buf[long(1.5f*i)+1]&0x0F)<<8;
        img_buf[i+1] = (byte_buf[long(1.5f*(i+1))]&0xF0)>>4 + byte_buf[long(1.5f*(i+1))+1]<<4;
    }

    cv::Mat bayer16(h, w, CV_16UC1, img_buf);
    cv::Mat bgr16(h, w, CV_16UC3);
    cv::cvtColor(bayer16, bgr16, cvt_code);
    cv::Mat bgr8(h, w, CV_8UC3);
    bgr16.convertTo(bgr8, CV_8UC3, 1.0f/256);
    cv::resize(bgr8, bgr8, cv::Size(w/4, h/4), 0, 0, cv::INTER_LINEAR);

#if 1
    cv::imshow(argv[4], bgr8);
    cv::waitKey(3000);
#endif
    free(byte_buf);
    free(img_buf);

    return 0;
}