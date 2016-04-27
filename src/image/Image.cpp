#include <image/Image.hpp>

#include <fstream>
#include <pfm/pfm_input_file.hpp>
#include <pic/pic_input_file.hpp>
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfArray.h>

#include <QImage>

namespace hdrv {

Image::Image(int w, int h, int c, Format f, std::vector<uint8_t>&& data)
  : width_(w)
  , height_(h)
  , channels_(c)
  , format_(f)
  , data_(std::move(data))
{}

Image Image::makeEmpty()
{
  std::vector<uint8_t> data(1, 0);
  return Image(0, 0, 1, Byte, std::move(data));
}

Result<Image> Image::loadPFM(std::string const& path)
{
  try {
    std::ifstream stream(path, std::ios::binary);
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

Result<Image> Image::loadPIC(std::string const& path)
{
  try {
    std::ifstream stream(path, std::ios::binary);
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

Result<Image> Image::loadEXR(std::string const& path)
{
  try {
    Imf::RgbaInputFile file(path.c_str());
    auto dw = file.dataWindow();
    int w = dw.max.x - dw.min.x + 1;
    int h = dw.max.y - dw.min.y + 1;
    int c = 4;

    std::vector<uint8_t> data(w * h * c * sizeof(float));
    float * d = reinterpret_cast<float *>(data.data());

    Imf::Array2D<Imf::Rgba> pixels(1, w);
    for (int y = 0; y < h; ++y, ++dw.min.y) {
      file.setFrameBuffer(&pixels[0][0] - dw.min.x - dw.min.y * w, 1, w);
      file.readPixels(dw.min.y);
      for (int x = 0; x < w; ++x) {
        auto const& p = pixels[0][x];
        int index = (h - y - 1) * w * c + x * c; // flip horizontally
        d[index + 0] = (float)p.r;
        d[index + 1] = (float)p.g;
        d[index + 2] = (float)p.b;
        d[index + 3] = (float)p.a;
      }
    }
    return Result<Image>(Image(w, h, c, Float, std::move(data)));

  } catch (std::exception const& e) {
    return Result<Image>(std::string("OpenEXR loader: ") + e.what());
  }
}

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