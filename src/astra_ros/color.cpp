#include "color.hpp"

using namespace astra_ros;

namespace
{
  // HSV / RGB conversion code
  // Adapted from:
  // https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
  static Hsv rgb2hsv(const Rgb &in);
  static Rgb hsv2rgb(const Hsv &in);

  Hsv rgb2hsv(const Rgb &in)
  {
    Hsv out;

    float min = in.r < in.g ? in.r : in.g;
    min = min < in.b ? min : in.b;

    float max = in.r > in.g ? in.r : in.g;
    max = max > in.b ? max : in.b;

    out.v = max;                                // v
    const float delta = max - min;
    if (delta < 0.0001f)
    {
      out.s = 0;
      out.h = 0; // undefined, maybe nan?
      return out;
    }

    if (max > 0.0)
    {
      // NOTE: if Max is == 0, this divide would cause a crash
      out.s = delta / max; // s
    }
    else
    {
      // if max is 0, then r = g = b = 0              
      // s = 0, h is undefined
      out.s = 0.0;
      out.h = NAN; // its now undefined
      return out;
    }

    if (in.r >= max)
    {
      out.h = (in.g - in.b) / delta; // between yellow & magenta
    }
    else if (in.g >= max)
    {
      out.h = 2.0f + (in.b - in.r) / delta; // between cyan & yellow
    }
    else
    {
      out.h = 4.0f + (in.r - in.g) / delta; // between magenta & cyan        
    }

    out.h *= 60.0f; // degrees

    if (out.h < 0.0f)
    {
      out.h += 360.0f;
    }

    return out;
  }


  Rgb hsv2rgb(const Hsv &in)
  {
    Rgb out;

    if (in.s <= 0.0f)
    {
      out.r = in.v;
      out.g = in.v;
      out.b = in.v;
      return out;
    }

    float hh = in.h;
    if(hh >= 360.0f) hh = 0.0f;
    hh /= 60.0f;

    const long i = (long)hh;
    const float ff = hh - i;
    const float p = in.v * (1.0f - in.s);
    const float q = in.v * (1.0f - (in.s * ff));
    const float t = in.v * (1.0f - (in.s * (1.0f - ff)));

    switch(i)
    {
      case 0:
      {
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
      }
      case 1:
      {
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
      }
      case 2:
      {
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;
      }
      case 3:
      {
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
      }
      case 4:
      {
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
      }
      case 5:
      default:
      {
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
      }
    }
    
    return out;     
  }

  const static float GOLDEN_ANGLE = 137.507764f;

  float goldenHue(const std::size_t i)
  {
    return fmod(GOLDEN_ANGLE * i, 360.0f);
  }
}

Rgba::Rgba()
  : r(0.0f)
  , g(0.0f)
  , b(0.0f)
  , a(1.0f)
{
}

Rgba::Rgba(const float r, const float g, const float b, const float a)
  : r(r)
  , g(g)
  , b(b)
  , a(a)
{
}

Rgba::operator std_msgs::ColorRGBA () const noexcept
{
  std_msgs::ColorRGBA ret;
  ret.r = r;
  ret.g = g;
  ret.b = b;
  ret.a = a;
  return ret;
}

const Rgba Rgba::RED(1.0f, 0.0f, 0.0f, 1.0f);
const Rgba Rgba::GREEN(0.0f, 1.0f, 0.0f, 1.0f);
const Rgba Rgba::BLUE(0.0f, 0.0f, 1.0f, 1.0f);
const Rgba Rgba::WHITE(1.0f, 1.0f, 1.0f, 1.0f);
const Rgba Rgba::BLACK(0.0f, 0.0f, 0.0f, 1.0f);

Rgba Rgba::fromRgb(const Rgb &rgb)
{
  return fromRgb(rgb, 1.0);
}

Rgba Rgba::fromRgb(const Rgb &rgb, const float a)
{
  Rgba ret;
  ret.a = a;
  ret.r = rgb.r;
  ret.g = rgb.g;
  ret.b = rgb.b;
  return ret;
}

Rgba Rgba::fromHsv(const Hsv &hsv)
{
  return fromRgb(Rgb::fromHsv(hsv));
}

Rgb::Rgb()
  : r(0.0f)
  , g(0.0f)
  , b(0.0f)
{
}

Rgb::Rgb(const float r, const float g, const float b)
  : r(r)
  , g(g)
  , b(b)
{
}

Rgb::operator std_msgs::ColorRGBA () const noexcept
{
  std_msgs::ColorRGBA ret;
  ret.r = r;
  ret.g = g;
  ret.b = b;
  ret.a = 1.0f;
  return ret;
}

Rgb::operator Rgba () const noexcept
{
  return toRgba();
}

const Rgb Rgb::RED(1.0f, 0.0f, 0.0f);
const Rgb Rgb::GREEN(0.0f, 1.0f, 0.0f);
const Rgb Rgb::BLUE(0.0f, 0.0f, 1.0f);
const Rgb Rgb::WHITE(1.0f, 1.0f, 1.0f);
const Rgb Rgb::BLACK(0.0f, 0.0f, 0.0f);

Hsv Rgb::toHsv() const
{
  return rgb2hsv(*this);
}

Rgba Rgb::toRgba() const
{
  return Rgba::fromRgb(*this);
}

Rgba Rgb::toRgba(const float a) const
{
  return Rgba::fromRgb(*this, a);
}

Rgb Rgb::fromHsv(const Hsv &hsv)
{
  return hsv2rgb(hsv);
}

Hsv::Hsv()
  : h(0.0f)
  , s(0.0f)
  , v(0.0f)
{
}

Hsv::Hsv(const float h, const float s, const float v)
  : h(h)
  , s(s)
  , v(v)
{
}

Rgb Hsv::toRgb() const
{
  return hsv2rgb(*this);
}

Rgba Hsv::toRgba() const
{
  return toRgb().toRgba();
}

Rgba Hsv::toRgba(const float a) const
{
  return toRgb().toRgba();
}

Hsv Hsv::fromRgb(const Rgb &rgb)
{
  return rgb2hsv(rgb);
}

Hsv astra_ros::indexColor(const std::size_t i)
{
  return Hsv(goldenHue(i), 1.0f, 1.0f);
}