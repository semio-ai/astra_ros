#ifndef _ASTRA_ROS_COLOR_HPP_
#define _ASTRA_ROS_COLOR_HPP_

#include <cstdint>

#include <std_msgs/ColorRGBA.h>

namespace astra_ros
{
  struct Rgba;
  struct Rgb;
  struct Hsv;

  /**
   * \class Rgba
   * \brief A RGB color with alpha component
   */
  struct Rgba
  {
    /**
     * Create a fully transparent black color
     */
    Rgba();

    /**
     * Construct a Rgba
     * \param r The red component (0.0 .. 1.0)
     * \param g The green component (0.0 .. 1.0)
     * \param b The blue component (0.0 .. 1.0)
     * \param a The alpha component (0.0 .. 1.0)
     */
    Rgba(const float r, const float g, const float b, const float a);

    operator std_msgs::ColorRGBA () const noexcept;

    // Red [0.0 .. 1.0] (inclusive)
    float r;
    // Green [0.0 .. 1.0] (inclusive)
    float g;
    // Blue [0.0 .. 1.0] (inclusive)
    float b;
    // Alpha [0.0 .. 1.0] (inclusive)
    float a;

    static const Rgba RED;
    static const Rgba GREEN;
    static const Rgba BLUE;
    static const Rgba WHITE;
    static const Rgba BLACK;
    static const Rgba TRANSPARENT;

    static Rgba fromHsv(const Hsv &hsv);
    static Rgba fromRgb(const Rgb &rgb);
    static Rgba fromRgb(const Rgb &rgb, const float a);
  };

  /**
   * \class Rgb
   */
  struct Rgb
  {
    Rgb();
    Rgb(const float r, const float g, const float b);

    operator std_msgs::ColorRGBA () const noexcept;
    operator Rgba () const noexcept;

    // Red [0.0 .. 1.0] (inclusive)
    float r;
    // Green [0.0 .. 1.0] (inclusive)
    float g;
    // Blue [0.0 .. 1.0] (inclusive)
    float b;

    static const Rgb RED;
    static const Rgb GREEN;
    static const Rgb BLUE;
    static const Rgb WHITE;
    static const Rgb BLACK;

    Hsv toHsv() const;
    Rgba toRgba() const;
    Rgba toRgba(const float a) const;
    static Rgb fromHsv(const Hsv &hsv);
  };

  

  struct Hsv
  {
    Hsv();
    Hsv(const float h, const float s, const float v);

    // Hue angle in degrees
    float h;
    // Saturation [0.0 .. 1.0] (inclusive)
    float s;
    // Value [0.0 .. 1.0] (inclusive)
    float v;

    Rgb toRgb() const;
    Rgba toRgba() const;
    Rgba toRgba(const float a) const;
    static Hsv fromRgb(const Rgb &rgb);
  };

  /**
   * Returns a unique color for a given integer i
   */
  Hsv indexColor(const std::size_t i);
}

#endif