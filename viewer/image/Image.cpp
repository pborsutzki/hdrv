#include <image/Image.hpp>

#include <fstream>

#include <pfm/pfm_input_file.hpp>
#include <pfm/pfm_output_file.hpp>

#include <pic/pic_input_file.hpp>
#include <pic/pic_output_file.hpp>


#ifdef _WIN32
# pragma warning(push)
#  pragma warning(disable: 4800) //warning C4800 : 'int' : forcing value to bool 'true' or 'false' (performance warning)
#endif // _WIN32
#  include <OpenEXR/ImfInputFile.h>
#  include <OpenEXR/ImfRgbaFile.h>
#  include <OpenEXR/ImfChannelList.h>
#  include <OpenEXR/ImfArray.h>
#ifdef _WIN32
# pragma warning(pop)
#endif // _WIN32

#include <QImage>

namespace hdrv {

Image::Image(int w, int h, int c, Format f, std::vector<uint8_t>&& data)
  : width_(w)
  , height_(h)
  , layers_({ {"default", {"R", "G", "B", "A" }, f, std::move(data)} })
{
  layers_.begin()->channels.resize(c);
}

Image::Image(int w, int h, std::vector<Layer>&& layers)
  : width_(w)
  , height_(h)
  , layers_(std::move(layers))
{}

Image Image::makeEmpty()
{
  std::vector<uint8_t> data(1, 0);
  return Image(0, 0, 1, Byte, std::move(data));
}

float Image::value(int x, int y, int channel, int layer) const
{
  auto i = ((height() - y - 1) * width() + x) * channels(layer) + channel;
  if (layers_[layer].format == Float) {
    float result;
    memcpy(&result, data(layer) + i * sizeof(float), sizeof(float));
    return result;
  } else {
    return (float)layers_[layer].data[i];
  }
}

Result<Image> Image::scaleByHalf() const
{
  // scales default layer only
  const int layer = 0;

  // Looses some pixels for odd resolutions

  if (width_ <= 1 && height_ <= 1) {
    return Result<Image>("Image is too small for further downscaling by half.");
  }

  int newWidth = std::max(width_ / 2, 1);
  int newHeight = std::max(height_ / 2, 1);

  std::vector<uint8_t> newdata(newWidth * newHeight * channels(layer) * pixelSizeInBytes(layer));
  if (format(layer) == Float)
  {
    for (int x = 0; x < newWidth; ++x)
      for (int y = 0; y < newHeight; ++y)
        for (int c = 0; c < channels(layer); ++c)
        {
          float avg = 0.f;
          for (int xo = 0; xo < 2; ++xo)
            for (int yo = 0; yo < 2; ++yo)
            {
              avg += value(
                std::min(x * 2 + xo, width_ - 1),
                std::max(height_ - y * 2 - yo - 1, 0),
                c, layer);
            }

          const int outputOffset =
            y * channels(layer) * newWidth +
            x * channels(layer) +
            c;

          avg /= 4.f;
          memcpy(newdata.data() + outputOffset * sizeof(float), &avg, sizeof(float));
        }
  }
  else
  {
    return Result<Image>("Scaling non-floating-point images is not supported.");
  }

  return Result<Image>(Image(newWidth, newHeight, channels(layer), format(layer), std::move(newdata)));
}

// PFM

Result<Image> Image::loadPFM(std::string const& path)
{
  try {
    std::ifstream stream(path, std::ios::binary);
    return loadPFM(stream);
  }
  catch (std::exception const& e) {
    return Result<Image>(std::string("PFM loader: ") + e.what());
  }
}

Result<Image> Image::loadPFM(std::istream & stream)
{
  try {
    pfm::pfm_input_file file(stream);

    pfm::format_type format;
    size_t width, height;
    pfm::byte_order_type byteOrder;
    double scale;
    file.read_header(format, width, height, byteOrder, scale);
    int c = format == pfm::color_format ? 3 : 1;
    int w = (int)width;
    int h = (int)height;

    std::vector<uint8_t> data(w * h * c * sizeof(float));
    float* d = reinterpret_cast<float*>(data.data());
    for (int y = 0; y < h; ++y) {
      if (format == pfm::color_format) {
        file.read_color_scanline(reinterpret_cast<pfm::color_pixel *>(&d[y * w * c]), width);
      } else {
        file.read_grayscale_scanline(reinterpret_cast<pfm::grayscale_pixel *>(&d[y * w * c]), width);
      }
    }

    return Result<Image>(Image(w, h, c, Float, std::move(data)));

  } catch (std::exception const& e) {
    return Result<Image>(std::string("PFM loader: ") + e.what());
  }
}

Result<bool> Image::storePFM(std::string const& path, int layer) const
{
  if (format(layer) != Float) {
    return Result<bool>("Cannot store LDR image as HDR image.");
  }
  try {
    std::ofstream stream(path, std::ios::binary);
    pfm::pfm_output_file file(stream);
    file.write_header(pfm::color_format, width(), height(), pfm::host_byte_order, 1.0);

    std::unique_ptr<pfm::color_pixel[]> scanline(new pfm::color_pixel[width()]);
    for (int y = height() - 1; y >= 0; --y) {
      for (int x = 0; x < width(); ++x) {
        scanline[x][0] = value(x, y, 0, layer);
        scanline[x][1] = value(x, y, 1, layer);
        scanline[x][2] = value(x, y, 2, layer);
      }
      file.write_color_scanline(scanline.get(), width());
    }
    return Result<bool>(true);

  } catch (std::exception const& e) {
    return Result<bool>(std::string("PFM export failed: ") + e.what());
  }
}

// Radiance PIC

Result<Image> Image::loadPIC(std::string const& path)
{
  try {
    std::ifstream stream(path, std::ios::binary);
    return loadPIC(stream);
  }
  catch (std::exception const& e) {
    return Result<Image>(std::string("Radiance PIC loader: ") + e.what());
  }
}

Result<Image> Image::loadPIC(std::istream & stream)
{
  try {
    pic::pic_input_file file(stream);

    pic::format_type format;
    double exposure;
    file.read_information_header(format, exposure);
    if (format != pic::_32_bit_rle_rgbe) {
      return Result<Image>("Radiance PIC loader: format not supported.");
    }

    pic::resolution_string_type resolutionType;
    size_t width, height;
    file.read_resolution_string(resolutionType, width, height);
    if (resolutionType != pic::neg_y_pos_x) {
      return Result<Image>("Radiance PIC loader: resolution type not supported.");
    }
    int w = (int)width;
    int h = (int)height;

    std::vector<uint8_t> data(w * h * 3 * sizeof(float));
    float* d = reinterpret_cast<float*>(data.data());
    std::unique_ptr<pic::pixel[]> scanline(new pic::pixel[w]);
    for (int y = 0; y < h; ++y) {
      file.read_scanline(scanline.get(), w);
      for (int x = 0; x < w; ++x) {
        int index = (h - y - 1) * 3 * w + x * 3;
        pic::rgbe_to_rgb(scanline[x][0], scanline[x][1], scanline[x][2], scanline[x][3],
          d[index], d[index + 1], d[index + 2]);
      }
    }

    return Result<Image>(Image(w, h, 3, Float, std::move(data)));

  } catch (std::exception const& e) {
    return Result<Image>(std::string("Radiance PIC loader: ") + e.what());
  }
}

Result<bool> Image::storePIC(std::string const& path, int layer) const
{
  if (format(layer) != Float) {
    return Result<bool>("Cannot store LDR image as HDR image.");
  }
  try {
    std::ofstream stream(path, std::ios::binary);
    pic::pic_output_file file(stream);

    file.write_information_header(pic::_32_bit_rle_rgbe, 1.0);
    file.write_resolution_string(pic::neg_y_pos_x, width(), height());

    std::unique_ptr<pic::pixel[]> scanline(new pic::pixel[width()]);
    for (int y = 0; y < height(); ++y) {
      for (int x = 0; x < width(); ++x) {
        pic::rgb_to_rgbe(value(x, y, 0, layer), value(x, y, 1, layer), value(x, y, 2, layer),
          scanline[x][0], scanline[x][1], scanline[x][2], scanline[x][3]);
      }
      file.write_scanline(scanline.get(), width());
    }
    return Result<bool>(true);

  } catch (std::exception const& e) {
    return Result<bool>(std::string("Radiance PIC export failed: ") + e.what());
  }
}

// ILM OpenEXR

template<typename Source>
Result<Image> loadEXRFile(Source && source)
{
  try {
    Imf::InputFile file(std::forward<Source>(source));
    auto dw = file.header().dataWindow();
    int w = dw.max.x - dw.min.x + 1;
    int h = dw.max.y - dw.min.y + 1;
    auto &channels = file.header().channels();

    std::map<std::string, std::vector<std::string>> layers;
    for (auto it = channels.begin(); it != channels.end(); ++it)
    {
      const std::string name = it.name();
      std::string layerName = "";
      std::string channelName = name;

      size_t pos = name.find_last_of('.');
      if (pos != std::string::npos) {
        layerName = name;
        layerName.resize(pos+1); // layername plus dot
        channelName = name.substr(pos+1); // channelname without dot
      }

      auto layerMapIt = layers.find(layerName);
      if (layerMapIt != layers.end()) {
        layerMapIt->second.push_back(channelName);
      } else {
        layers.emplace(layerName, std::vector<std::string>({ channelName }));
      }
    }

    static const std::vector<std::string> channelOrder = { "U", "V", "X", "Y", "Z", "W", "C", "R", "G", "B", "A" };

    std::for_each(layers.begin(), layers.end(), [](std::pair<const std::string, std::vector<std::string>> &layers) {
      auto & channels = layers.second;
      std::vector<std::string> sortedChannels;
      for (auto const& order : channelOrder) {
        auto it = std::find(channels.begin(), channels.end(), order);
        if (it != channels.end()) {
          channels.erase(it);
          sortedChannels.push_back(order);
        }
      }
      sortedChannels.insert(sortedChannels.cend(), channels.cbegin(), channels.cend());
      channels = sortedChannels;
    });

    std::vector<Image::Layer> layerData;
    for (auto &layer : layers) {
      std::string const &layerName = layer.first;
      int channelCount = (int)layer.second.size();

      const std::string channelNames = std::accumulate(layer.second.cbegin(), layer.second.cend(), std::string());

      Imf::FrameBuffer fb;
      layerData.emplace_back(Image::Layer({layerName + channelNames, layer.second, Image::Float, std::vector<uint8_t>(w * h * channelCount * sizeof(float))}));
      char *basePtr = reinterpret_cast<char*>(layerData.back().data.data());

      for (int channel = 0; channel < layer.second.size(); ++channel)
      {
        size_t xStride = channelCount * sizeof(float);
        size_t yStride = xStride * w;
        char* channelBasePtr = basePtr + sizeof(float) * channel + yStride * (h - 1); // last line
        size_t yStrideInv = ~(size_t)0 - yStride + 1;
        fb.insert(layer.first + layer.second[channel], Imf::Slice(Imf::FLOAT, channelBasePtr, xStride, yStrideInv));
      }
      file.setFrameBuffer(fb);
      file.readPixels(dw.min.y, dw.max.y);
    }

    return Result<Image>(Image(w, h, std::move(layerData)));
  } catch (std::exception const& e) {
    return Result<Image>(std::string("OpenEXR loader: ") + e.what());
  }
}

Result<Image> Image::loadEXR(Imf::IStream & stream)
{
  return loadEXRFile(stream);
}

Result<Image> Image::loadEXR(std::string const& path)
{
  return loadEXRFile(path.c_str());
}

Result<bool> Image::storeEXR(std::string const& path, int layer) const
{
  try {
    Imf::RgbaOutputFile file(path.c_str(), width(), height(), Imf::WRITE_RGBA);

    std::unique_ptr<Imf::Rgba[]> scanline(new Imf::Rgba[width()]);
    for (int y = 0; y < height(); ++y) {
      for (int x = 0; x < width(); ++x) {
        auto & p = scanline[x];
        p.r = half(value(x, y, 0, layer));
        p.g = channels(layer) > 1 ? half(value(x, y, 1, layer)) : p.r;
        p.b = channels(layer) > 2 ? half(value(x, y, 2, layer)) : p.r;
        p.a = channels(layer) > 3 ? half(value(x, y, 3, layer)) : half(1.0f);
      }
      file.setFrameBuffer(scanline.get(), 1, 0);
      file.writePixels();
    }
    return Result<bool>(true);

  } catch (std::exception const& e) {
    return Result<bool>(std::string("OpenEXR export failed: ") + e.what());
  }
}

// Qt LDR Image

Result<Image> Image::loadImage(std::string const& path)
{
  QImage img;
  if (img.load(path.c_str())) {
    img = img.convertToFormat(img.hasAlphaChannel() ? QImage::Format_RGBA8888 : QImage::Format_RGB888).mirrored();

    int w = img.width();
    int h = img.height();
    int c = img.hasAlphaChannel() ? 4 : 3;

    std::vector<uint8_t> data(w * h * c);
    std::copy(img.bits(), img.bits() + w*h*c, data.begin());
    return Result<Image>(Image(w, h, c, Byte, std::move(data)));
  } else {
    return Result<Image>(std::string("Image loader failed."));
  }
}

}
