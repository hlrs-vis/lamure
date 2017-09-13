#ifndef LAMURE_SPARSEOCTREE_H
#define LAMURE_SPARSEOCTREE_H

#include <lamure/pro/data/DenseCache.h>
#include <lamure/pro/partitioning/entities/OctreeNode.h>
#include <lamure/pro/partitioning/interfaces/Partitionable.h>

namespace prov
{
class SparseOctree : public OctreeNode
{
  public:
    class Builder
    {
      public:
        Builder() {}
        Builder *from(DenseCache &dense_cache)
        {
            this->_dense_cache = &dense_cache;
            return this;
        }
        Builder *from(string &input_path)
        {
            this->_input_path = &input_path;
            return this;
        }
        Builder *with_sort(Sort sort)
        {
            this->_sort = sort;
            return this;
        }
        Builder *with_max_depth(uint8_t max_depth)
        {
            this->_max_depth = max_depth;
            return this;
        }
        Builder *with_min_per_node(uint8_t min_per_node)
        {
            this->_min_per_node = min_per_node;
            return this;
        }
        ~Builder() {}
        SparseOctree build()
        {
            SparseOctree octree(0, _sort, _max_depth, _min_per_node);

            if(_dense_cache != nullptr)
            {
                _glue_pairs();
                octree._pair_ptrs.reserve(_unsorted_pairs.size());
                for(size_t i = 0; i < _unsorted_pairs.size(); i++)
                {
                    octree._pair_ptrs.push_back(s_ptr<dense_pair>(&_unsorted_pairs.at(i)));
                }
                octree.partition();
            }
            else if(_input_path != nullptr)
            {
                // TODO
            }

            return octree;
        }

      private:
        DenseCache *_dense_cache = nullptr;
        string *_input_path = nullptr;

        Sort _sort = STD_SORT;
        uint8_t _max_depth = 10;
        uint8_t _min_per_node = 1;

        vec<pair<prov::DensePoint, prov::DenseMetaData>> _unsorted_pairs;

        void _glue_pairs()
        {
            printf("\nStart gluing pairs\n");
            for(uint64_t i = 0; i < _dense_cache->get_points().size(); i++)
            {
                dense_pair pair(_dense_cache->get_points().at(i), _dense_cache->get_points_metadata().at(i));
                this->_unsorted_pairs.push_back(pair);
            }
            printf("\nEnd gluing pairs\n");
        }
    };

  public:
    SparseOctree() : OctreeNode() {}
    SparseOctree(uint8_t _depth) : OctreeNode(_depth) {}
    SparseOctree(uint8_t _depth, Sort _sort, uint8_t _max_depth, uint8_t _min_per_node) : OctreeNode(_depth, _sort, _max_depth, _min_per_node) {}

    void debug_information_loss(DenseCache &dense_cache, uint64_t num_probes)
    {
        printf("\nEnter debug information loss\n");
        float information_loss = 0;
        num_probes = std::min(num_probes, dense_cache.get_points().size());
        for(uint64_t i = 0; i < num_probes; i++)
        {
            size_t ind = (size_t)(((float)rand() / RAND_MAX) * dense_cache.get_points().size());
            OctreeNode *node_ptr = lookup_node_at_position(dense_cache.get_points().at(ind).get_position());
            if(node_ptr == nullptr)
            {
                vec3f pos = dense_cache.get_points().at(ind).get_position();
                printf("\nnullptr hit during lookup: %f, %f, %f\n", pos.x, pos.y, pos.z);
                throw new std::runtime_error("\nnullptr hit during lookup\n");
            }
            //                        else
            //                        {
            //                            printf("\ndepth returned: %u\n", (*node_ptr).get_depth());
            //                        }
            information_loss += compare_metadata((*node_ptr).get_aggregate_metadata(), dense_cache.get_points_metadata().at(ind)) / (float)num_probes;
            //            printf("\nIntermediate information loss: %lf\n", information_loss);
        }
        printf("\nFinal information loss: %lf%%\n", information_loss * 100);
    }

    OctreeNode *lookup_node_at_position(vec3f position)
    {
        //        printf("\nEnter lookup node at position\n");

        if(!fits_in_boundaries(position))
        {
            //            printf("\nPosition does not fit into boundaries\n");
            return this;
        }

        if(this->_partitions.empty())
        {
            //            printf("\nMaximum depth reached at this position\n");
            return this;
        }

        for(uint8_t i = 0; i < this->_partitions.size(); i++)
        {
            //            printf("\nEnter partition lookup\n");
            OctreeNode *node = this->_partitions.at(i).lookup_node_at_position(position);
            if(node != nullptr)
            {
                return node;
            }
        }

        return this;
    }

    void debug_randomized_lookup(uint64_t num_probes)
    {
        for(uint64_t i = 0; i < num_probes; i++)
        {
            float rand_x = rand() / (float)RAND_MAX * (this->_max.x - this->_min.x) + this->_min.x;
            float rand_y = rand() / (float)RAND_MAX * (this->_max.y - this->_min.y) + this->_min.y;
            float rand_z = rand() / (float)RAND_MAX * (this->_max.z - this->_min.z) + this->_min.z;
            vec3f rand_pos(rand_x, rand_y, rand_z);
            OctreeNode *node_ptr = lookup_node_at_position(rand_pos);
            if(node_ptr == nullptr)
            {
                printf("\nnullptr hit during lookup: %f, %f, %f\n", rand_pos.x, rand_pos.y, rand_pos.z);
                throw new std::runtime_error("\nnullptr hit during lookup\n");
            }
        }
    }

  protected:
    void partition()
    {
        printf("\nStart partitioning\n");

        OctreeNode::partition();

        printf("\nEnd partitioning\n");
    }

  private:
    // TODO: see to completion
    /*static void output_tree(SparseOctree octree, string output_path)
    {
        ofstream ofstream_tree(output_path);
        text_oarchive oa_tree(ofstream_tree);
        oa_tree << octree;
    }

    static SparseOctree load_tree(string _input_path)
    {
        SparseOctree octree;
        ifstream ifstream_tree(_input_path);
        text_iarchive ia_tree(ifstream_tree);
        ia_tree >> octree;
        return octree;
    }*/

    float compare_metadata(const DenseMetaData &data, const DenseMetaData &ref_data)
    {
        float information_loss = 0;
        information_loss += std::abs(data.get_photometric_consistency() - ref_data.get_photometric_consistency());
        //        information_loss += std::abs(int(data.get_images_seen().size()) - int(ref_data.get_images_seen().size())) / (double)(1 + int(ref_data.get_images_seen().size()));
        //        information_loss += std::abs(int(data.get_images_not_seen().size()) - int(ref_data.get_images_not_seen().size())) / (double)(1 + int(ref_data.get_images_not_seen().size()));
        return information_loss; // / 3;
    }
};
};
#endif // LAMURE_SPARSEOCTREE_H
