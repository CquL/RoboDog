#ifndef X30_PLANE_SEG_CORE__TEST__SUPPORT__SHA256_HPP_
#define X30_PLANE_SEG_CORE__TEST__SUPPORT__SHA256_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace x30_plane_seg_core
{
namespace test_support
{

using Sha256Digest = std::array<std::uint8_t, 32>;

class Sha256
{
public:
  Sha256() noexcept;

  void update(const void * data, std::size_t size) noexcept;
  Sha256Digest finish() noexcept;

private:
  void transform(const std::uint8_t * block) noexcept;

  std::array<std::uint32_t, 8> state_;
  std::array<std::uint8_t, 64> block_{};
  std::size_t buffered_bytes_{0};
  std::uint64_t total_bytes_{0};
};

Sha256Digest sha256(const void * data, std::size_t size) noexcept;

}  // namespace test_support
}  // namespace x30_plane_seg_core

#endif  // X30_PLANE_SEG_CORE__TEST__SUPPORT__SHA256_HPP_
