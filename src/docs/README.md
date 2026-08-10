# Project documentation

Only active operating and architecture documents live in this directory.
Historical implementation notes are available through Git history and pull
requests instead of version-numbered files in the runtime tree.

## Start here

- [`REPOSITORY_STRUCTURE.md`](REPOSITORY_STRUCTURE.md): module ownership,
  high-frequency code, and PR boundaries.
- [`FULL_GRASP_PIPELINE.md`](FULL_GRASP_PIPELINE.md): current perception-to-grasp
  data path and safety gates.
- [`RPLIDAR_AND_HEALTHCHECK.md`](RPLIDAR_AND_HEALTHCHECK.md): RPLIDAR C1 setup
  and read-only startup checks.
- [`ROARM_JOINT_SELFTEST.md`](ROARM_JOINT_SELFTEST.md): operator-supervised
  J1-J4 motion verification.
- [`ROARM_HOME.md`](ROARM_HOME.md): custom T=122 startup home.
- [`ROARM_MANUAL_TEST_AND_CALIBRATION.md`](ROARM_MANUAL_TEST_AND_CALIBRATION.md):
  isolated Cartesian and coordinate validation.
- [`TISSUE_DATASET_CAPTURE.md`](TISSUE_DATASET_CAPTURE.md): image collection.
- [`CALIBRATION_AND_EXPERIMENT_LOGGING.md`](CALIBRATION_AND_EXPERIMENT_LOGGING.md):
  no-motion 3 x 3 capture, measured command-model result, structured logs,
  annotation, and error statistics.

## Documentation rule

Update an existing active document when behavior changes. Do not add package
manifests, ZIP installation notes, `V<n>` documents, or one-file fix reports;
the PR description and commit history are the revision record.
