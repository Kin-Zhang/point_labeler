#pragma once

#include <stdint.h>
#include <QtCore/QDir>
#include <QtCore/QString>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "common.h"
#include "data/kitti_utils.h"

/** \brief tile-based KITTI reader.
 *
 *  Given a size of a tile, the reader reads all scans that potentially overlap with the tile.
 *  If a scan overlaps with the tile is determined by a circle to square overlap test.
 *
 *  Thus, it might happen that the circle of radius max_distance overlaps with the square defined by the tile,
 *  but there are actually no points inside the tile. However, we cannot check this at this point in time,
 *  since we would have to open all point clouds and this would take forever.
 *
 *  \see settings.cfg
 *
 *  \author behley
 */

class DMReader {
 public:
  struct Tile {
    int32_t i, j;                   // tile coordinates
    std::vector<uint32_t> indexes;  // scan indexes
    float x, y, size;               // actual world coordinates.
  };

  /** \brief get poses and filenames of velodyne, labels, etc.
   *  If the labels directory does not exist, the directory is created.
   **/
  void initialize(const QString& directory);

  /** \brief number of scans. **/
  uint32_t count() const { return pcd_filenames_.size(); }

  void setMaximumDistance(float distance) { maxDistance_ = distance; }

  /** \brief get points, labels, and images for given world coordinates. **/
  void retrieve(const Eigen::Vector3f& position, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                std::vector<LabelsPtr>& labels, std::vector<std::string>& images);

  /** \brief get points, labels, and images for given indexes. **/
  void retrieve(uint32_t i, uint32_t j, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                std::vector<LabelsPtr>& labels, std::vector<std::string>& images);

  /** \brief update labels for given scan indexes. **/
  void update(const std::vector<uint32_t>& indexes, std::vector<LabelsPtr>& labels);

  void setTileSize(float size);

  const std::vector<Tile>& getTiles() const { return tiles_; }

  const Tile& getTile(const Eigen::Vector3f& pos) const;
  const Tile& getTile(uint32_t i, uint32_t j) const;

  const Eigen::Vector2i& numTiles() const { return numTiles_; }

  const std::vector<Eigen::Vector2f>& getTileTrajectory() const { return trajectory_; }

  std::map<uint32_t, uint32_t> getMaxInstanceIds() const { return maxInstanceIds_; }

  void updateMetaInformation(const std::map<uint32_t, uint32_t>& maxInstanceIds);

 protected:
  void readPoints(const std::string& filename, Laserscan& scan);
  void readLabels(const std::string& filename, std::vector<uint32_t>& labels);
  void readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses);

  QDir base_dir_;
  std::vector<Eigen::Matrix4f> poses_;
  std::vector<std::string> pcd_filenames_;
  std::vector<std::string> label_filenames_;
  std::vector<std::string> image_filenames_;

  // cache reads from before.
  std::map<uint32_t, PointcloudPtr> pointsCache_;
  std::map<uint32_t, LabelsPtr> labelCache_;

  float maxDistance_{15.0f};

  inline uint32_t tileIdxToOffset(uint32_t i, uint32_t j) const { return i + j * numTiles_.x(); }

  float tileSize_{50};
  std::vector<Tile> tiles_;
  Eigen::Vector2f offset_;
  Eigen::Vector2i numTiles_;

  std::vector<Eigen::Vector2f> trajectory_;

  std::map<uint32_t, uint32_t> maxInstanceIds_;
};

namespace impl
{
enum class PCDName { UNKNOWN, X, Y, Z, INTENSITY, RGB, LABEL, VALUE };

enum class PCDType { I, U, F };

enum class PCDDataType { ASCII, BINARY, BINARY_COMPRESSED };

struct PCDPointField {
	PCDName     name{PCDName::UNKNOWN};
	std::size_t size;
	std::size_t count{1};
	PCDType     type;
};

struct PCDHeader {
 public:
	std::vector<PCDPointField> fields;
	std::size_t                width;
	std::size_t                height;
	PCDDataType                datatype;
	Eigen::Matrix4f			   viewpoint;
};


inline std::string toLower(std::string str)
{
	std::transform(std::begin(str), std::end(str), std::begin(str),
	               [](auto c) { return std::tolower(c); });
	return str;
}
inline std::vector<std::string> splitString(std::string const& str, std::string const& splitters)
{
	std::vector<std::string> result;
	auto it = std::find_if_not(std::cbegin(str), std::cend(str), [&splitters](auto c) {
		return std::any_of(std::cbegin(splitters), std::cend(splitters),
		                   [c](auto e) { return c == e; });
	});
	for (auto last = std::cend(str); it != last;) {
		auto cur = it;
		it       = std::find_if(it, last, [&splitters](auto c) {
      return std::any_of(std::cbegin(splitters), std::cend(splitters),
			                         [c](auto e) { return c == e; });
    });
		result.emplace_back(cur, it);
		it = std::find_if_not(it, last, [&splitters](auto c) {
			return std::any_of(std::cbegin(splitters), std::cend(splitters),
			                   [c](auto e) { return c == e; });
		});
	}
	return result;
}

template <typename T>
T unpackASCIIPCDElement(std::string const& str, PCDType type, std::size_t size)
{
	switch (type) {
		case PCDType::I: return static_cast<T>(std::stoll(str));
		case PCDType::U: return static_cast<T>(std::stoull(str));
		case PCDType::F: return static_cast<T>(std::stod(str));
	}

	return T();
}

template <typename T>
T unpackBinaryPCDElement(char const* data, PCDType type, std::size_t size)
{
	switch (type) {
		case PCDType::I:
			switch (size) {
				case 1: {
					std::int8_t d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				case 2: {
					std::int16_t d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				case 4: {
					std::int32_t d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				case 8: {
					std::int64_t d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				default: return T();
			}
		case PCDType::U:
			switch (size) {
				case 1: {
					std::uint8_t d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				case 2: {
					std::uint16_t d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				case 4: {
					std::uint32_t d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				case 8: {
					std::uint64_t d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				default: return T();
			}
		case PCDType::F:
			switch (size) {
				case 4: {
					float d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				case 8: {
					double d;
					std::memcpy(&d, data, sizeof(d));
					return static_cast<T>(d);
				}
				default: return T();
			}
	}
	return T();
}

inline bool startsWith(const std::string& fullString, const std::string& starting) {
    if (fullString.length() >= starting.length()) {
        return fullString.compare(0, starting.length(), starting) == 0;
    } else {
        return false;
    }
}

inline PCDHeader readHeader(std::ifstream& file){
  PCDHeader header;
  std::string line;

  // Read header
	while (std::getline(file, line)) {
		if ('#' == line[0]) {
			// Skip comments
			continue;
		} else if (startsWith(line, "VERSION")) {
			// Do not care about the version
			continue;
		} else if (startsWith(line, "FIELDS")) {
			std::istringstream iss(line.substr(7));
			std::string        s;
			std::size_t        index{};
			while (iss >> s) {
				if (header.fields.size() == index) {
					header.fields.emplace_back();
				}

				s = toLower(s);

				if ("x" == s) {
					header.fields[index].name = PCDName::X;
				} else if ("y" == s) {
					header.fields[index].name = PCDName::Y;
				} else if ("z" == s) {
					header.fields[index].name = PCDName::Z;
				} else if ("intensity" == s) {
					header.fields[index].name = PCDName::INTENSITY;
				} else if ("rgb" == s) {
					header.fields[index].name = PCDName::RGB;
				} else if ("label" == s) {
					header.fields[index].name = PCDName::LABEL;
				} else if ("value" == s) {
					header.fields[index].name = PCDName::VALUE;
				} else {
					header.fields[index].name = PCDName::UNKNOWN;
				}
				++index;
			}
    } else if (startsWith(line, "SIZE")) {
			std::istringstream iss(line.substr(5));
			std::size_t        s;
			std::size_t        index{};
			while (iss >> s) {
				if (header.fields.size() == index) {
					header.fields.emplace_back();
				}

				header.fields[index].size = s;
				++index;
			}
		} else if (startsWith(line, "TYPE")) {
			std::istringstream iss(line.substr(5));
			char               s;
			std::size_t        index{};
			while (iss >> s) {
				if (header.fields.size() == index) {
					header.fields.emplace_back();
				}

				if ('I' == s) {
					header.fields[index].type = PCDType::I;
				} else if ('U' == s) {
					header.fields[index].type = PCDType::U;
				} else if ('F' == s) {
					header.fields[index].type = PCDType::F;
				}
				++index;
			}
		} else if (startsWith(line, "COUNT")) {
			std::istringstream iss(line.substr(6));
			std::size_t        s;
			std::size_t        index{};
			while (iss >> s) {
				if (header.fields.size() == index) {
					header.fields.emplace_back();
				}

				header.fields[index].count = s;
				++index;
			}
		} else if (startsWith(line, "WIDTH")) {
			std::istringstream iss(line.substr(6));
			if (!(iss >> header.width)) {
				// TODO: Error
			}
		} else if (startsWith(line, "HEIGHT")) {
			std::istringstream iss(line.substr(7));
			if (!(iss >> header.height)) {
				// TODO: Error
			}
		} else if (startsWith(line, "VIEWPOINT")) {
			std::istringstream iss(line.substr(10));
			header.viewpoint = Eigen::Matrix4f::Identity();
			double 		  x, y, z, qw, qx, qy, qz;
			if (!(iss >> x >> y >> z >> qw >> qx >> qy >> qz)) {
				// TODO Error
			}
			header.viewpoint.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
			header.viewpoint.block<3, 3>(0, 0) = Eigen::Quaternionf(qw, qx, qy, qz).toRotationMatrix();
			
		} else if (startsWith(line, "POINTS")) {
      // Skip since we can get this from width * height
    } else if (startsWith(line, "DATA")) {
			auto data = line.substr(5);  // FIXME: Trim
			if ("ascii" == data) {
				header.datatype = PCDDataType::ASCII;
			} else if ("binary" == data) {
				header.datatype = PCDDataType::BINARY;
			} else if ("binary_compressed" == data) {
				header.datatype = PCDDataType::BINARY_COMPRESSED;
			} else {
				// TODO: Handle error
			}
			break;  // Everything after is data
		}
  }
  return header;
}

}