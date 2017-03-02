/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/common/RelativeResourceRetriever.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"

namespace dart {
namespace common {

//==============================================================================
RelativeResourceRetriever::RelativeResourceRetriever(
    const Uri& baseUri,
    const ResourceRetrieverPtr& retriever)
  : mBaseUri(baseUri),
    mRetriever(retriever)
{
  if(!mRetriever)
    mRetriever = std::make_shared<LocalResourceRetriever>();
}

//==============================================================================
bool RelativeResourceRetriever::exists(const Uri& uri)
{
  return mRetriever->exists(generateRelativeUri(uri));
}

//==============================================================================
ResourcePtr RelativeResourceRetriever::retrieve(const Uri& uri)
{
  return mRetriever->retrieve(generateRelativeUri(uri));
}

//==============================================================================
Uri RelativeResourceRetriever::generateRelativeUri(const Uri& uri)
{
//  if(uri.mScheme.get_value_or("") == "")
//    return Uri::createFromRelativeUri(mBaseUri, uri.mPath.get_value_or(""));

//  if(!uri.mScheme)
//  {
//    const std::string s = uri.toString();
//    if(s.at(0) != '/')
//      return Uri::createFromRelativeUri(mBaseUri, s);
//  }

  if(uri.mAuthority && (!uri.mPath || uri.mPath.get().empty()))
  {
    return Uri::createFromRelativeUri(mBaseUri, uri.mAuthority.get());
  }

  return uri;
}

} // namespace common
} // namespace dart
