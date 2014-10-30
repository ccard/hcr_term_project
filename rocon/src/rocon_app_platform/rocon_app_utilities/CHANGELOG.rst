Changelog
=========

0.7.2 (2014-08-25)
------------------
* it updates its indices when it adds or removes repo
* bugfix for `#260 <https://github.com/robotics-in-concert/rocon_app_platform/issues/260>`_ `#253 <https://github.com/robotics-in-concert/rocon_app_platform/issues/253>`_
* now indexer works fine. rapp stores resources path in yaml_data
* now it uses normalised path and provides more debug messages
* add optional arguments on rocon_app cmd tool closes issue `#259 <https://github.com/robotics-in-concert/rocon_app_platform/issues/259>`_
* improve top-level arg parser logic. Now it does not validate the full launch xml though.
* update comments. absolute to relative
* remove debug message
* It marks invalid rapp if it contains tuple based resource. and give error
* update rapp spec document
* relative path working
* in the middle of change to relative path
* add simple indicator
* multiple chirp working
* parses parameters from file
* 0.7.1
* Merge branch 'indigo' into hydro-devel
* rocon_app_utilities: error: unconfigured build_depend on 'rocon_python_utils rocon_uri
* Contributors: Daniel Stonier, Jihoon Lee

0.7.0 (2014-05-06)
------------------
* command line suite for rapp indexing, caching, dependency installing.
* remote repository index caching.
* local ros package path rapp indexer.
* Contributors: Jihoon Lee
