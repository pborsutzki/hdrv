#pragma once

#include <string>
#include <memory>
#include <vector>
#include <boost/optional.hpp>

#include <OpenEXR/ImfIO.h>

namespace hdrv {

template<typename T>
class Result
{
public:
  operator bool() const { return (bool)value_; }
  T const& value() const& { return value_.get(); }
  T && value() && { return std::move(value_.get()); }
  std::string const& error() const { return error_; }

  Result(T && v) : value_(std::move(v)) {}
  Result(std::string const& error) : error_(error) {}

private:
  std::string error_;
  boost::optional<T> value_;
};

class Image
{
public:
  enum Format { Byte, Float };

  struct Layer
  {
    std::string name;
    std::vector<std::string> channels;
    Format format;
    std::vector<uint8_t> data;
  };

  static Image makeEmpty();

  static Result<Image> loadPFM(std::string const& path);
  static Result<Image> loadPIC(std::string const& path);
  static Result<Image> loadEXR(std::string const& path);
  static Result<Image> loadImage(std::string const& path);

  static Result<Image> loadPFM(std::istream & stream);
  static Result<Image> loadPIC(std::istream & stream);
  static Result<Image> loadEXR(Imf::IStream & stream);

  int width() const { return width_; }
  int height() const { return height_; }
  int channels(int layer) const { return (int)layers_.at(layer).channels.size(); }
  int pixelSizeInBytes(int layer) const { return layers_.at(layer).format == Byte ? sizeof(uint8_t) : sizeof(float); }
  int sizeInBytes(int layer) const { return (int)(width_ * height_ * layers_.at(layer).channels.size() * pixelSizeInBytes(layer)); }
  Format format(int layer) const { return layers_.at(layer).format; }
  uint8_t const* data(int layer) const { return layers_.at(layer).data.data(); }
  float value(int x, int y, int channel, int layer) const;
  size_t layerCount() const { return layers_.size(); }
  std::string const& layerName(int layer) { return layers_.at(layer).name; }
  std::string const& channelName(int layer, int channel) { return layers_.at(layer).channels.at(channel); }

  Result<bool> storePFM(std::string const& path, int layer) const;
  Result<bool> storePIC(std::string const& path, int layer) const;
  Result<bool> storeEXR(std::string const& path, int layer) const;

  Result<Image> scaleByHalf() const;

  Image(int w, int h, int c, Format f, std::vector<uint8_t>&& data);
  Image(int w, int h, std::vector<Layer>&& layers);

private:
  int width_;
  int height_;
  std::vector<Layer> layers_;
};

}