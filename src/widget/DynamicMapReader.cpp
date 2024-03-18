/**
 * @author: Qingwen Zhang (https://kin-zhang.github.io/)
 * @date: 2024-03-18, 20:41
 * 
 * Copyright (C) 20224, RPL, KTH Royal Institute of Technology
 * @details: This is the implementation of the DynamicMap Benchmark dataset format.
 */

#include <stdint.h>
#include "widget/DynamicMapReader.h"

#include <QtCore/QDir>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <sstream>

#include "rv/string_utils.h"

void DMReader::initialize(const QString& directory) {
  pcd_filenames_.clear();
  label_filenames_.clear();
  poses_.clear();

  pointsCache_.clear();
  labelCache_.clear();
  tiles_.clear();

  base_dir_ = QDir(directory);
  QDir pcd_dir(base_dir_.filePath("pcd"));
  QStringList entries = pcd_dir.entryList(QDir::Files, QDir::Name);
  entries.sort();
  for (int32_t i = 0; i < entries.size(); ++i)
    pcd_filenames_.push_back(pcd_dir.filePath(entries.at(i)).toStdString());

  std::cout << "Found " << pcd_filenames_.size() << " pcd files. in " << pcd_dir.path().toStdString() << std::endl;

  // create label dir, etc.
  QDir labels_dir(base_dir_.filePath("labels"));

  // find corresponding label files.
  if (!labels_dir.exists()) base_dir_.mkdir("labels");

  for (uint32_t i = 0; i < pcd_filenames_.size(); ++i) {
    QString filename = QFileInfo(QString::fromStdString(pcd_filenames_[i])).baseName() + ".label";
    // read pcd file to get the number of points.
    std::ifstream file;
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    file.imbue(std::locale());
    file.open(pcd_filenames_[i], std::ios_base::in | std::ios_base::binary);
    impl::PCDHeader header = impl::readHeader(file);
    file.close();

    uint32_t num_points = header.width * header.height;
    poses_.push_back(header.viewpoint);

    if (!labels_dir.exists(filename)) {
      std::ofstream out(labels_dir.filePath(filename).toStdString().c_str());
      std::vector<uint32_t> labels(num_points, 0);
      out.write(reinterpret_cast<const char*>(labels.data()), num_points * sizeof(uint32_t));
      out.close();
    }

    label_filenames_.push_back(labels_dir.filePath(filename).toStdString());
  }

  // FIXME: delete this later.
  std::string missing_img = QDir::currentPath().toStdString() + "/../assets/missing.png";
  QDir image_dir(base_dir_.filePath("image_2"));
  for (uint32_t i = 0; i < pcd_filenames_.size(); ++i) {
    QString filename = QFileInfo(QString::fromStdString(pcd_filenames_[i])).baseName() + ".png";
    if (image_dir.exists(filename)) {
      image_filenames_.push_back(image_dir.filePath(filename).toStdString());
    } else {
      image_filenames_.push_back(missing_img);
    }
  }

  // assumes that (0,0,0) is always the start.
  Eigen::Vector2f min = Eigen::Vector2f::Zero();
  Eigen::Vector2f max = Eigen::Vector2f::Zero();

  for (uint32_t i = 0; i < poses_.size(); ++i) {
    Eigen::Vector4f t = poses_[i].col(3);

    min.x() = std::min(t.x() - maxDistance_, min.x());
    min.y() = std::min(t.y() - maxDistance_, min.y());
    max.x() = std::max(t.x() + maxDistance_, max.x());
    max.y() = std::max(t.y() + maxDistance_, max.y());
  }

  //  std::cout << "tileSize = " << tileSize_ << std::endl;
  //  std::cout << "min = " << min << ", max = " << max << std::endl;

  offset_.x() = std::ceil((std::abs(min.x()) - 0.5 * tileSize_) / tileSize_) * tileSize_ + 0.5 * tileSize_;
  offset_.y() = std::ceil((std::abs(min.y()) - 0.5 * tileSize_) / tileSize_) * tileSize_ + 0.5 * tileSize_;

  //  std::cout << "offset = " << offset_ << std::endl;

  numTiles_.x() = std::ceil((std::abs(min.x()) - 0.5 * tileSize_) / tileSize_) +
                  std::ceil((max.x() - 0.5 * tileSize_) / tileSize_) + 1;
  numTiles_.y() = std::ceil((std::abs(min.y()) - 0.5 * tileSize_) / tileSize_) +
                  std::ceil((max.y() - 0.5 * tileSize_) / tileSize_) + 1;

  //  std::cout << "numTiles = " << numTiles_ << std::endl;

  tiles_.clear();
  tiles_.resize(numTiles_.x() * numTiles_.y());

  Eigen::Vector2f idxRadius(maxDistance_ / tileSize_, maxDistance_ / tileSize_);

  for (uint32_t i = 0; i < uint32_t(numTiles_.x()); ++i) {
    for (uint32_t j = 0; j < uint32_t(numTiles_.y()); ++j) {
      auto& tile = tiles_[tileIdxToOffset(i, j)];

      tile.i = i;
      tile.j = j;
      tile.x = i * tileSize_ - offset_.x() + 0.5 * tileSize_;
      tile.y = j * tileSize_ - offset_.y() + 0.5 * tileSize_;
      tile.size = tileSize_;
    }
  }

  trajectory_.clear();

  Eigen::Vector2f e(0.5 * tileSize_, 0.5 * tileSize_);
  for (uint32_t i = 0; i < poses_.size(); ++i) {
    Eigen::Vector2f t = poses_[i].col(3).head(2);
    Eigen::Vector2f idx((t.x() + offset_.x()) / tileSize_, (t.y() + offset_.y()) / tileSize_);

    trajectory_.push_back(Eigen::Vector2f((t.x() + offset_.x()) / tileSize_, (t.y() + offset_.y()) / tileSize_));

    //    tiles_[tileIdxToOffset(uint32_t(idx.x()), uint32_t(idx.y()))].indexes.push_back(i);
    //    uint32_t u_min = std::max(int32_t(idx.x() - idxRadius.x()), 0);
    //    uint32_t u_max = std::min(int32_t(std::ceil(idx.x() + idxRadius.x())), numTiles_.x());
    //    uint32_t v_min = std::max(int32_t(idx.y() - idxRadius.y()), 0);
    //    uint32_t v_max = std::min(int32_t(std::ceil(idx.y() + idxRadius.y())), numTiles_.y());

    // FIXME: workaround check all tiles.
    for (uint32_t u = 0; u < uint32_t(numTiles_.x()); ++u) {
      for (uint32_t v = 0; v < uint32_t(numTiles_.y()); ++v) {
        auto& tile = tiles_[tileIdxToOffset(u, v)];
        Eigen::Vector2f q = t - Eigen::Vector2f(tile.x, tile.y);
        q[0] = std::abs(q[0]);
        q[1] = std::abs(q[1]);

        // check for exact overlap (see Behley et al., ICRA, 2015)
        if (std::max(q[0], q[1]) > e[0] + maxDistance_) continue;  // definitely outside.
        if (std::min(q[0], q[1]) < e[0] || (q - e).norm() < maxDistance_) {
          tile.indexes.push_back(i);
        }
      }
    }
  }

  // sanity check:

  for (auto& t : tiles_) {
    std::sort(t.indexes.begin(), t.indexes.end());
    //    std::cout << "Tile has " << t.indexes.size() << " tiles associated." << std::endl;
    for (uint32_t i = 1; i < t.indexes.size(); ++i) {
      if (t.indexes[i - 1] == t.indexes[i]) {
        std::cout << "found duplicate!" << std::endl;
      }
    }
  }

  uint32_t tileCount = 0;
  for (uint32_t i = 0; i < uint32_t(numTiles_.x()); ++i) {
    for (uint32_t j = 0; j < uint32_t(numTiles_.y()); ++j) {
      auto& tile = tiles_[tileIdxToOffset(i, j)];

      std::sort(tile.indexes.begin(), tile.indexes.end());
      if (tile.indexes.size() > 0) tileCount += 1;
    }
  }

  std::cout << "#tiles  = " << tileCount << std::endl;

  // meta information for faster loading.
  if (base_dir_.exists("instances.txt")) {
    std::cout << "Reading instances.txt..." << std::flush;

    std::ifstream in(base_dir_.filePath("instances.txt").toStdString());

    while (in.good()) {
      std::string line;
      std::getline(in, line);
      if (line.size() == 0) break;

      std::vector<std::string> tokens = rv::split(line, ":");
      if (tokens.size() != 2) {
        throw std::runtime_error("Invalid instance meta information found!");
      }

      uint32_t label = boost::lexical_cast<uint32_t>(tokens[0]);
      uint32_t maxInstanceId = boost::lexical_cast<uint32_t>(tokens[1]);
      maxInstanceIds_[label] = maxInstanceId;

      in.peek();
    }

    in.close();
    std::cout << "finished." << std::endl;
  } else {
    std::cout << "Generating intances.txt" << std::flush;
    // get the counts from the label files.
    for (const std::string& filename : label_filenames_) {
      std::vector<uint32_t> labels;
      readLabels(filename, labels);

      for (uint32_t instance_label : labels) {
        uint32_t instanceId = (instance_label >> 16) & uint32_t(0xFFFF);
        uint32_t label = instance_label & uint32_t(0xFFFF);
        if (maxInstanceIds_.find(label) == maxInstanceIds_.end())
          maxInstanceIds_[label] = instanceId;
        else
          maxInstanceIds_[label] = std::max(instanceId, maxInstanceIds_[label]);
      }
    }

    // directly update meta information:
    updateMetaInformation(maxInstanceIds_);
  }

}

void DMReader::updateMetaInformation(const std::map<uint32_t, uint32_t>& maxInstanceIds) {
  std::ofstream out(base_dir_.filePath("instances.txt").toStdString().c_str());
  for (auto it = maxInstanceIds.begin(); it != maxInstanceIds.end(); ++it) {
    out << it->first << ":" << it->second << std::endl;
  }
  out.close();
}

void DMReader::retrieve(const Eigen::Vector3f& position, std::vector<uint32_t>& indexes,
                           std::vector<PointcloudPtr>& points, std::vector<LabelsPtr>& labels,
                           std::vector<std::string>& images) {
  Eigen::Vector2f idx((position.x() + offset_.x()) / tileSize_, (position.y() + offset_.y()) / tileSize_);

  retrieve(idx.x(), idx.y(), indexes, points, labels, images);
}

void DMReader::retrieve(uint32_t i, uint32_t j, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                           std::vector<LabelsPtr>& labels, std::vector<std::string>& images) {
  indexes.clear();
  points.clear();
  labels.clear();
  images.clear();

  std::vector<int32_t> indexesBefore;
  for (auto it = pointsCache_.begin(); it != pointsCache_.end(); ++it) indexesBefore.push_back(it->first);
  std::vector<int32_t> indexesAfter;

  uint32_t scansRead = 0;

  indexes = tiles_[tileIdxToOffset(i, j)].indexes;
  std::sort(indexes.begin(), indexes.end());
  for (uint32_t t : indexes) {
    indexesAfter.push_back(t);
    if (pointsCache_.find(t) == pointsCache_.end()) {
      scansRead += 1;

      points.push_back(std::shared_ptr<Laserscan>(new Laserscan));
      readPoints(pcd_filenames_[t], *points.back());
      // std::cout << "Reading " << pcd_filenames_[t] << std::endl;
      pointsCache_[t] = points.back();
      points.back()->pose = poses_[t];

      labels.push_back(std::shared_ptr<std::vector<uint32_t>>(new std::vector<uint32_t>()));
      readLabels(label_filenames_[t], *labels.back());
      labelCache_[t] = labels.back();

      if (points.back()->size() != labels.back()->size()) {
        std::cout << "Filename: " << pcd_filenames_[t] << std::endl;
        std::cout << "Filename: " << label_filenames_[t] << std::endl;
        std::cout << "num. points = " << points.back()->size() << " vs. num. labels = " << labels.back()->size()
                  << std::endl;
        throw std::runtime_error("Inconsistent number of labels.");
      }

    } else {
      points.push_back(pointsCache_[t]);
      labels.push_back(labelCache_[t]);
    }

    images.push_back(image_filenames_[t]);
  }

  std::cout << scansRead << " point clouds read." << std::endl;

  // FIXME: keep more scans in cache. not only remove unloaded scans.

  std::sort(indexesBefore.begin(), indexesBefore.end());
  std::sort(indexesAfter.begin(), indexesAfter.end());

  std::vector<int32_t> needsDelete(indexesBefore.size());
  std::vector<int32_t>::iterator end = std::set_difference(
      indexesBefore.begin(), indexesBefore.end(), indexesAfter.begin(), indexesAfter.end(), needsDelete.begin());

  for (auto it = needsDelete.begin(); it != end; ++it) {
    pointsCache_.erase(*it);
    labelCache_.erase(*it);
  }
}

const DMReader::Tile& DMReader::getTile(const Eigen::Vector3f& position) const {
  Eigen::Vector2f idx((position.x() + offset_.x()) / tileSize_, (position.y() + offset_.y()) / tileSize_);
  return tiles_[tileIdxToOffset(idx.x(), idx.y())];
}
const DMReader::Tile& DMReader::getTile(uint32_t i, uint32_t j) const { return tiles_[tileIdxToOffset(i, j)]; }

void DMReader::setTileSize(float size) { tileSize_ = size; }

void DMReader::update(const std::vector<uint32_t>& indexes, std::vector<LabelsPtr>& labels) {

  for (uint32_t i = 0; i < indexes.size(); ++i) {
    if (labels[i]->size() == 0) {
      std::cout << "Warning: 0 labels?" << std::endl;
      continue;
    }

    if (pointsCache_.find(indexes[i]) == pointsCache_.end()) {
      std::cout << "Warning: labels of non cached points?" << std::endl;
      continue;
    }

    if (labels[i]->size() != pointsCache_[indexes[i]]->size()) {
      std::cout << "Warning: inconsistent numbers of labels for given point cloud!" << std::endl;

      continue;
    }

    if (label_filenames_.size() < indexes[i]) {
      std::cout << "Warning: wrong index?" << std::endl;

      continue;
    }

    std::ofstream out(label_filenames_[indexes[i]].c_str());
    out.write((const char*)&(*labels[i])[0], labels[i]->size() * sizeof(uint32_t));
    out.close();
  }
}


void DMReader::readPoints(const std::string& filename, Laserscan& scan) {
  // std::cout << "Reading " << filename << std::endl;
	std::ifstream file;
  std::string line;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	file.imbue(std::locale());
	file.open(filename, std::ios_base::in | std::ios_base::binary);
  impl::PCDHeader header = impl::readHeader(file);
	std::size_t point_size{};
	for (auto const& f : header.fields) {
		point_size += f.size * f.count;
	}

	scan.clear();
  uint32_t num_points = header.width * header.height;
  std::vector<Point3f>& points = scan.points;
  std::vector<float>& remissions = scan.remissions;
	points.resize(num_points);
  remissions.resize(num_points);

	switch (header.datatype) {
		case impl::PCDDataType::ASCII: {
			for (uint32_t pi = 0; pi < num_points; ++pi) {
				if (!std::getline(file, line)) {
					// TODO: Handle error
				}

				auto s = impl::splitString(line, "\t\r\n ");

				if (s.size() != header.fields.size()) {
					// TODO: Handle error
				}

				for (std::size_t i{}; auto const& field : header.fields) {
					if (1 != field.count) {
						i += field.count;
						continue;  // Cannot handle anything other than one count
					}

					switch (field.name) {
						case impl::PCDName::X:
							points[pi].x = impl::unpackASCIIPCDElement<float>(s[i], field.type, field.size);
							break;
						case impl::PCDName::Y:
							points[pi].y = impl::unpackASCIIPCDElement<float>(s[i], field.type, field.size);
							break;
						case impl::PCDName::Z:
							points[pi].z = impl::unpackASCIIPCDElement<float>(s[i], field.type, field.size);
							break;
						case impl::PCDName::INTENSITY:
							remissions[pi] = impl::unpackASCIIPCDElement<float>(s[i], field.type, field.size);
							break;
						case impl::PCDName::UNKNOWN: break;
					}

					i += field.count;
				}
			}
			break;
		}
		case impl::PCDDataType::BINARY: {
			std::unique_ptr<char[]> buffer(new char[point_size]);
			for (uint32_t pi = 0; pi < num_points; ++pi) {
				if (!file.read(buffer.get(), point_size)) {
					// TODO: Handle error
				}

				for (auto data = buffer.get(); auto const& field : header.fields) {
					if (1 != field.count) {
						data += field.count * field.size;
						continue;  // Cannot handle anything other than one count
					}

					switch (field.name) {
						case impl::PCDName::X:
							points[pi].x = impl::unpackBinaryPCDElement<float>(data, field.type, field.size);
							break;
						case impl::PCDName::Y:
							points[pi].y = impl::unpackBinaryPCDElement<float>(data, field.type, field.size);
							break;
						case impl::PCDName::Z:
							points[pi].z = impl::unpackBinaryPCDElement<float>(data, field.type, field.size);
							break;
						case impl::PCDName::INTENSITY:
              remissions[pi] = impl::unpackBinaryPCDElement<float>(data, field.type,
								                                                        field.size);
							break;
						case impl::PCDName::UNKNOWN: break;
					}

					data += field.count * field.size;
				}
			}
			break;
		}
	}

  file.close();
  // std::cout << "Read " << num_points << " points." << std::endl;
}

void DMReader::readLabels(const std::string& filename, std::vector<uint32_t>& labels) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "Unable to open label file. " << std::endl;
    return;
  }

  labels.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (sizeof(uint32_t));
  in.seekg(0, std::ios::beg);

  labels.resize(num_points);
  in.read((char*)&labels[0], num_points * sizeof(uint32_t));

  in.close();
}
