/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pointpillar.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cuda_runtime.h>
#include "NvInfer.h"
#include "NvOnnxConfig.h"
#include "NvOnnxParser.h"
#include "NvInferRuntime.h"
#include <ctime>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}

TRT::~TRT(void)
{
  std::cerr<<"TRT xigou"<<std::endl;
  delete(context_);
  delete(engine_);
  checkCudaErrors(cudaEventDestroy(start_));
  checkCudaErrors(cudaEventDestroy(stop_));
  return;
}

TRT::TRT(std::string modelFile, cudaStream_t stream):stream_(stream)
{
  clock_t start_cpu = clock();
  std::string modelCache = modelFile + ".cache";
  std::fstream trtCache(modelCache, std::ifstream::in);
  checkCudaErrors(cudaEventCreate(&start_));
  checkCudaErrors(cudaEventCreate(&stop_));
  time_t t = time(0); 
  char date[32]={NULL};
  char time[32]={NULL};
  strftime(date, sizeof(date), "%Y-%m-%d",localtime(&t));
  strftime(time, sizeof(time), "%H:%M:%S",localtime(&t));
  std::string date_=date;
  std::string time_=time;
  std::string status_filepath="/home/nvidia/datalog/adas/system_log/"+date_;
  std::string command_status;
  if (0 != access(status_filepath.c_str(), 0))
    {
      // 返回 0 表示创建成功，-1 表示失败
      command_status = "mkdir -p " + status_filepath;
      system(command_status.c_str());    	
    }
  std::string status_filename=status_filepath+"/"+"status.txt";
  if (!trtCache.is_open())
  {
	  std::cout << "Building TRT engine."<<std::endl;
    // define builder
    auto builder = (nvinfer1::createInferBuilder(gLogger_));

    // define network
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = (builder->createNetworkV2(explicitBatch));

    // define onnxparser
    auto parser = (nvonnxparser::createParser(*network, gLogger_));
    if (!parser->parseFromFile(modelFile.data(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
    {
        std::cerr << ": failed to parse onnx model file, please check the onnx version and trt support op!"
                  << std::endl;
        exit(-1);
    }

    // define config
    auto networkConfig = builder->createBuilderConfig();
#if defined (__arm64__) || defined (__aarch64__) 
    networkConfig->setFlag(nvinfer1::BuilderFlag::kFP16);
    std::cout << "Enable fp16!" << std::endl;
#endif
    // set max batch size
    builder->setMaxBatchSize(1);
    // set max workspace
    networkConfig->setMaxWorkspaceSize(size_t(1) << 30);

    engine_ = (builder->buildEngineWithConfig(*network, *networkConfig));

    if (engine_ == nullptr)
    {
      std::cerr << ": engine init null!" << std::endl;
      exit(-1);
    }

    // serialize the engine, then close everything down
    auto trtModelStream = (engine_->serialize());
    std::fstream trtOut(modelCache, std::ifstream::out);
    if (!trtOut.is_open())
    {
       std::cout << "Can't store trt cache.\n";
       exit(-1);
    }

    trtOut.write((char*)trtModelStream->data(), trtModelStream->size());
    trtOut.close();
    trtModelStream->destroy();

    networkConfig->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();

  } else {
	  std::cout << "load TRT cache."<<std::endl;
    char *data;
    unsigned int length;

    // get length of file:
    trtCache.seekg(0, trtCache.end);
    length = trtCache.tellg();
    trtCache.seekg(0, trtCache.beg);

    data = (char *)malloc(length);
    if (data == NULL ) {
       std::cout << "Can't malloc data.\n";
       exit(-1);
    }

    
    trtCache.read(data, length);
    // create context
    auto runtime = nvinfer1::createInferRuntime(gLogger_);
    
    if (runtime == nullptr) {	  std::cout << "load TRT cache0."<<std::endl;
        std::cerr << ": runtime null!" << std::endl;
        exit(-1);
    }
    //plugin_ = nvonnxparser::createPluginFactory(gLogger_);
    engine_ = (runtime->deserializeCudaEngine(data, length, 0));
    if (engine_ == nullptr) {
        std::cerr << ": engine null!" << std::endl;
        exit(-1);
    }
    free(data);
    trtCache.close();
  }

  context_ = engine_->createExecutionContext();
  std::ofstream ofs;
  ofs.open(status_filename,std::ios::app);
  if (ofs.is_open()){
      ofs << time_ << ": ";
      ofs << "\n";
      ofs << "  Model loads in normal condition.\n";
      
  }
  ofs.close();
  clock_t finish_cpu = clock();
  double elapsed_secs = static_cast<double>(finish_cpu - start_cpu) / CLOCKS_PER_SEC;
  std::cerr << "TIME: load: "<<elapsed_secs << " s" <<std::endl;
  return;
}

int TRT::doinfer(void**buffers)
{
  int status;
  
  status = context_->enqueueV2(buffers, stream_, &start_);

  if (!status)
  {
      return -1;
  }
  
  return 0;
}

PointPillar::PointPillar(std::string modelFile, cudaStream_t stream):stream_(stream)
{
  clock_t start_cpu = clock();
  checkCudaErrors(cudaEventCreate(&start_));
  checkCudaErrors(cudaEventCreate(&stop_));

  pre_.reset(new PreProcessCuda(stream_));
  trt_.reset(new TRT(modelFile, stream_));
  post_.reset(new PostProcessCuda(stream_));

  //point cloud to voxels
  voxel_features_size_ = MAX_VOXELS * params_.max_num_points_per_pillar * 4 * sizeof(float);
  voxel_num_size_ = MAX_VOXELS * sizeof(unsigned int);
  voxel_idxs_size_ = MAX_VOXELS* 4 * sizeof(unsigned int);

  checkCudaErrors(cudaMallocManaged((void **)&voxel_features_, voxel_features_size_));
  checkCudaErrors(cudaMallocManaged((void **)&voxel_num_, voxel_num_size_));
  checkCudaErrors(cudaMallocManaged((void **)&voxel_idxs_, voxel_idxs_size_));

  checkCudaErrors(cudaMemsetAsync(voxel_features_, 0, voxel_features_size_, stream_));
  checkCudaErrors(cudaMemsetAsync(voxel_num_, 0, voxel_num_size_, stream_));
  checkCudaErrors(cudaMemsetAsync(voxel_idxs_, 0, voxel_idxs_size_, stream_));

  //TRT-input
  features_input_size_ = MAX_VOXELS * params_.max_num_points_per_pillar * 10 * sizeof(float);
  checkCudaErrors(cudaMallocManaged((void **)&features_input_, features_input_size_));
  checkCudaErrors(cudaMallocManaged((void **)&params_input_, sizeof(unsigned int)));

  checkCudaErrors(cudaMemsetAsync(features_input_, 0, features_input_size_, stream_));
  checkCudaErrors(cudaMemsetAsync(params_input_, 0, sizeof(unsigned int), stream_));

  //output of TRT -- input of post-process
  cls_size_ = params_.feature_x_size * params_.feature_y_size * params_.num_classes * params_.num_anchors * sizeof(float);
  box_size_ = params_.feature_x_size * params_.feature_y_size * params_.num_box_values * params_.num_anchors * sizeof(float);
  dir_cls_size_ = params_.feature_x_size * params_.feature_y_size * params_.num_dir_bins * params_.num_anchors * sizeof(float);
  checkCudaErrors(cudaMallocManaged((void **)&cls_output_, cls_size_));
  checkCudaErrors(cudaMallocManaged((void **)&box_output_, box_size_));
  checkCudaErrors(cudaMallocManaged((void **)&dir_cls_output_, dir_cls_size_));

  //output of post-process
  bndbox_size_ = (params_.feature_x_size * params_.feature_y_size * params_.num_anchors * 9 + 1) * sizeof(float);
  checkCudaErrors(cudaMallocManaged((void **)&bndbox_output_, bndbox_size_));

  res_.reserve(100);
  clock_t finish_cpu = clock();
  double elapsed_secs = static_cast<double>(finish_cpu - start_cpu) / CLOCKS_PER_SEC;
  std::cout << "TIME: trt pointpillar: "<<elapsed_secs << " s" <<std::endl;
  return;
}

PointPillar::~PointPillar(void)
{
  std::cerr<<"PointPillar xigou"<<std::endl;
  pre_.reset();
  trt_.reset();
  post_.reset();

  checkCudaErrors(cudaFree(voxel_features_));
  checkCudaErrors(cudaFree(voxel_num_));
  checkCudaErrors(cudaFree(voxel_idxs_));

  checkCudaErrors(cudaFree(features_input_));
  checkCudaErrors(cudaFree(params_input_));

  checkCudaErrors(cudaFree(cls_output_));
  checkCudaErrors(cudaFree(box_output_));
  checkCudaErrors(cudaFree(dir_cls_output_));

  checkCudaErrors(cudaFree(bndbox_output_));

  checkCudaErrors(cudaEventDestroy(start_));
  checkCudaErrors(cudaEventDestroy(stop_));
  return;
}

int PointPillar::doinfer(void*points_data, unsigned int points_size, std::vector<Bndbox> &nms_pred, const float nms_pp_threshold, const float score_threshold)
{
#if PERFORMANCE_LOG
  float generateVoxelsTime = 0.0f;
  checkCudaErrors(cudaEventRecord(start_, stream_));
#endif

  pre_->generateVoxels((float*)points_data, points_size,
        params_input_,
        voxel_features_, 
        voxel_num_,
        voxel_idxs_);

#if PERFORMANCE_LOG
  checkCudaErrors(cudaEventRecord(stop_, stream_));
  checkCudaErrors(cudaDeviceSynchronize());
  checkCudaErrors(cudaEventElapsedTime(&generateVoxelsTime, start_, stop_));
  unsigned int params_input_cpu;
  checkCudaErrors(cudaMemcpy(&params_input_cpu, params_input_, sizeof(unsigned int), cudaMemcpyDefault));
  std::cout<<"find pillar_num: "<< params_input_cpu <<std::endl;
#endif

#if PERFORMANCE_LOG
  float generateFeaturesTime = 0.0f;
  checkCudaErrors(cudaEventRecord(start_, stream_));
#endif

  pre_->generateFeatures(voxel_features_,
      voxel_num_,
      voxel_idxs_,
      params_input_,
      features_input_);

#if PERFORMANCE_LOG
  checkCudaErrors(cudaEventRecord(stop_, stream_));
  checkCudaErrors(cudaEventSynchronize(stop_));
  checkCudaErrors(cudaEventElapsedTime(&generateFeaturesTime, start_, stop_));
#endif

#if PERFORMANCE_LOG
  float doinferTime = 0.0f;
  checkCudaErrors(cudaEventRecord(start_, stream_));
#endif

  void *buffers[] = {features_input_, voxel_idxs_, params_input_, cls_output_, box_output_, dir_cls_output_};
  trt_->doinfer(buffers);
  checkCudaErrors(cudaMemsetAsync(params_input_, 0, sizeof(unsigned int), stream_));

#if PERFORMANCE_LOG
  checkCudaErrors(cudaEventRecord(stop_, stream_));
  checkCudaErrors(cudaEventSynchronize(stop_));
  checkCudaErrors(cudaEventElapsedTime(&doinferTime, start_, stop_));
#endif

#if PERFORMANCE_LOG
  float doPostprocessCudaTime = 0.0f;
  checkCudaErrors(cudaEventRecord(start_, stream_));
#endif

  post_->doPostprocessCuda(cls_output_, box_output_, dir_cls_output_,
                          bndbox_output_,score_threshold);
  checkCudaErrors(cudaDeviceSynchronize());
  float obj_count = bndbox_output_[0];

  int num_obj = static_cast<int>(obj_count);
  auto output = bndbox_output_ + 1;
  float temp_res=res_[0].x;
  int temp;
  
  for (int i = 0; i < num_obj; i++) {
    auto Bb = Bndbox(output[i * 9],
                    output[i * 9 + 1], output[i * 9 + 2], output[i * 9 + 3],
                    output[i * 9 + 4], output[i * 9 + 5], output[i * 9 + 6],
                    static_cast<int>(output[i * 9 + 7]),
                    output[i * 9 + 8]);
    res_.push_back(Bb);
  }

  nms_cpu(res_, nms_pp_threshold, nms_pred);

  res_.clear();
  if (temp_res==res_[0].x)
  {
      temp=1;
  }
  else
  {
      temp=0;
  }
  //std::vector<Bndbox>().swap(res_);
  

#if PERFORMANCE_LOG
  checkCudaErrors(cudaDeviceSynchronize());
  checkCudaErrors(cudaEventRecord(stop_, stream_));
  checkCudaErrors(cudaEventSynchronize(stop_));
  checkCudaErrors(cudaEventElapsedTime(&doPostprocessCudaTime, start_, stop_));
  std::cout<<"TIME: generateVoxels: "<< generateVoxelsTime <<" ms." <<std::endl;
  std::cout<<"TIME: generateFeatures: "<< generateFeaturesTime <<" ms." <<std::endl;
  std::cout<<"TIME: doinfer: "<< doinferTime <<" ms." <<std::endl;
  std::cout<<"TIME: doPostprocessCuda: "<< doPostprocessCudaTime <<" ms." <<std::endl;

#endif

  return temp;
}
