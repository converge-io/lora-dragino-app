{
  description = "LoRa Dragino development environment";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-22.11";
  inputs.flake-utils.url = "github:numtide/flake-utils";

  outputs = { self, nixpkgs, flake-utils  }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
        };

        deps = with pkgs;
          [
            gcc-arm-embedded-9
            gnumake
            pre-commit
            uncrustify
            protobuf
            (pkgs.python3.withPackages (
              ps: with ps; [
                protobuf
                pyserial
              ]
            ))
          ];

        shell = pkgs.mkShell {
          name = "lora-dragino-hw-env";
          buildInputs = deps;
          shellHook = ''
            export TOOLCHAIN_PATH=${pkgs.gcc-arm-embedded-9}/bin/
            export TREMO_SDK_PATH=$PWD
          '';
        };


      in {
        # Used with nix develop
        devShell = shell;
      });
}
