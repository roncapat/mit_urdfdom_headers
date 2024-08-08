/* Author: Matthew Chignoli */

#ifndef URDF_INTERFACE_CLUSTER_H
#define URDF_INTERFACE_CLUSTER_H

#include "link.h"

namespace urdf{

class Cluster : public std::vector<LinkSharedPtr>
{
public:
    Cluster() { this->clear(); };

    std::string name;

    std::vector<ClusterSharedPtr> child_clusters;

    ClusterSharedPtr getParent() const
    {return parent_cluster_.lock();};

    void setParent(const ClusterSharedPtr &parent)
    { parent_cluster_ = parent; }

    void clear()
    {
        std::vector<LinkSharedPtr>::clear();
        this->name.clear();
        this->child_clusters.clear();
    };

private:
  ClusterWeakPtr parent_cluster_;

};

}

#endif
