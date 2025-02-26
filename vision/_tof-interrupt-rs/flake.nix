{
  description = "tof-rs cross compilation";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { nixpkgs, flake-utils, rust-overlay, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };
        rustToolchain = pkgs.rust-bin.stable."1.82.0".default.override {
          extensions = [ "rust-src" "rust-analyzer" ];
          targets = [ "aarch64-unknown-linux-gnu" ];
        };


        crossTools = with pkgs; [
          pkgsCross.aarch64-multiplatform.stdenv.cc
        ];

        nativeBuildInputs = with pkgs; [
          rustToolchain
          pkg-config
        ] ++ crossTools;

        buildInputs = with pkgs; [];

        darwinInputs = with pkgs; lib.optionals pkgs.stdenv.isDarwin [
          darwin.apple_sdk.frameworks.Security
          darwin.apple_sdk.frameworks.SystemConfiguration
          darwin.libobjc
        ];

      in {
        devShells.default = pkgs.mkShell {
          buildInputs = nativeBuildInputs ++ buildInputs ++ darwinInputs;

          RUST_SRC_PATH = "${rustToolchain}/lib/rustlib/src/rust/library";
          
          CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER = "${pkgs.pkgsCross.aarch64-multiplatform.stdenv.cc}/bin/aarch64-unknown-linux-gnu-gcc";
          
          PYO3_CROSS_PYTHON_VERSION = "3.11";
          shellHook = ''
            echo "VL53L1X development environment loaded with Rust 1.82.0"
            echo ""
            echo "cargo build --target aarch64-unknown-linux-gnu"
          '';
        };

        packages.default = pkgs.rustPlatform.buildRustPackage {
          pname = "vl53l1x-rs";
          version = "0.1.0";
          src = ./.;
          cargoLock.lockFile = ./Cargo.lock;
          
          doCheck = !pkgs.stdenv.isDarwin;
          
          buildInputs = buildInputs ++ darwinInputs;
          nativeBuildInputs = with pkgs; [ 
            rustToolchain
            pkg-config
          ] ++ crossTools;
          
          CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER = "${pkgs.pkgsCross.aarch64-multiplatform.stdenv.cc}/bin/aarch64-unknown-linux-gnu-gcc";
          
          meta = with pkgs.lib; {
            description = "tof-rs cross compilation";
            platforms = platforms.linux;
          };
        };
      }
    );
}
