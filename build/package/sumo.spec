#
# spec file for package sumo
#
# Copyright (C) 2001-2016 DLR (http://www.dlr.de/) and contributors
#  This file is part of SUMO.
#  SUMO is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
# norootforbuild
Name:           sumo
Summary:        Simulation of Urban Mobility - A Microscopic Traffic Simulation
Version:        svn
Release:        1
Url:            http://sumo.dlr.de/
Source0:        %{name}-src-%{version}.tar.gz
Source1:        %{name}-doc-%{version}.zip
Source2:        %{name}.desktop
Source3:        %{name}.png
Source4:        %{name}.xml
License:        GPL-3.0+
Group:          Productivity/Scientific/Other
BuildRoot:      %{_tmppath}/%{name}-%{version}-build
BuildRequires:  gcc-c++ unzip help2man
BuildRequires:  pkgconfig(x11) pkgconfig(xerces-c) pkgconfig(fox)
%if 0%{?fedora_version} || 0%{?suse_version}
BuildRequires:  fdupes
BuildRequires:  pkgconfig(gdal)
%endif
%if 0%{?fedora_version} || 0%{?centos_version} || 0%{?rhel_version} || 0%{?scientificlinux_version}
BuildRequires:  pkgconfig(xcursor) pkgconfig(xrandr) pkgconfig(xi) 
BuildRequires:  libGLU-devel libXext-devel libXft-devel libjpeg-devel libpng-devel libtiff-devel
%endif

Autoreqprov: on

%description
"Simulation of Urban MObility" (SUMO) is an open source,
highly portable, microscopic road traffic simulation package
designed to handle large road networks.

%prep
%setup -q
unzip -o %{SOURCE1} -d ..
mv docs/tutorial docs/examples
%if 0%{?sles_version}
find . -name "*.jar" | xargs rm
%endif

%build
%configure
%{__make}
%{__make} man

%install
%makeinstall
%__mkdir_p %{buildroot}%{_prefix}/lib/sumo
rm -rf tools/contributed/traci4j
cp -a tools data %{buildroot}%{_prefix}/lib/sumo
%__mkdir_p %{buildroot}%{_bindir}
%__ln_s ../../bin %{buildroot}%{_prefix}/lib/sumo
%__ln_s ../lib/sumo/tools/assign/duaIterate.py %{buildroot}%{_bindir}/duaIterate.py
%__ln_s ../lib/sumo/tools/osmWebWizard.py %{buildroot}%{_bindir}/osmWebWizard.py
%__ln_s ../lib/sumo/tools/randomTrips.py %{buildroot}%{_bindir}/randomTrips.py
%__ln_s ../lib/sumo/tools/traceExporter.py %{buildroot}%{_bindir}/traceExporter.py
install -d -m 755 %{buildroot}%{_mandir}/man1
install -p -m 644 docs/man/*.1 %{buildroot}%{_mandir}/man1
install -Dm644 %{SOURCE2} %{buildroot}%{_datadir}/applications/%{name}.desktop
install -Dm644 %{SOURCE3} %{buildroot}%{_datadir}/pixmaps/%{name}.png
%if 0%{?suse_version}
install -Dm644 %{SOURCE4} %{buildroot}%{_datadir}/mime/application/%{name}.xml
%fdupes -s docs
%fdupes %{buildroot}
%endif

%files
%defattr(-,root,root)
%{_bindir}/*
%{_prefix}/lib/sumo
%doc AUTHORS COPYING README ChangeLog docs/pydoc docs/userdoc docs/examples
%{_mandir}/man1/*
%{_datadir}/applications/%{name}.desktop
%{_datadir}/pixmaps/%{name}.png
%if 0%{?suse_version} > 1200
%{_datadir}/mime/application
%endif

%changelog
