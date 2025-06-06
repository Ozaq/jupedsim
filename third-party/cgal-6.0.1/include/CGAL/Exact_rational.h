// Copyright (c) 2014
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Number_types/include/CGAL/Exact_rational.h $
// $Id: include/CGAL/Exact_rational.h 50cfbde3b84 $
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Laurent Rineau

#ifndef CGAL_EXACT_RATIONAL_H
#define CGAL_EXACT_RATIONAL_H

#include <CGAL/Number_types/internal/Exact_type_selector.h>

namespace CGAL {

/*!
\ingroup nt_cgal

`Exact_rational` is an exact rational number type, constructible from `double`.

It is a typedef of another number type. Its exact definition depends on
the availability the third-party libraries \gmp, \core, and \leda. \cgal must
be configured with at least one of those libraries.

\cgalModels{Field,RealEmbeddable,Fraction,FromDoubleConstructible}

*/
#if DOXYGEN_RUNNING

typedef unspecified_type Exact_rational;

#else // not DOXYGEN_RUNNING

using Exact_rational = internal::Exact_NT_backend<internal::Default_exact_nt_backend>::Rational;

#endif

} /* end namespace CGAL */

#endif // CGAL_EXACT_RATIONAL_H
