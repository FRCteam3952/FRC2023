{
  description = "Development environment for FRC 2023";
  inputs = {
    devshell.url = "github:numtide/devshell";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };
  outputs = {
    devshell,
    nixpkgs,
    ...
  }: let
    genSystems = nixpkgs.lib.genAttrs ["x86_64-linux"];
  in {
    devShells = genSystems (system: let
      pkgs = nixpkgs.legacyPackages.${system};
      createAlias = newName: pkg: bin:
        pkgs.symlinkJoin {
          name = newName;
          paths = [(pkgs.writeShellScriptBin "${newName}" "${pkgs.${pkg}}/bin/${bin} $@") pkgs.${pkg}];
          buildInputs = [pkgs.makeWrapper];
          postBuild = "wrapProgram $out/bin/${newName} --prefix PATH : $out/bin";
        };
      createAliasSame = newName: pkg: createAlias newName pkg pkg;

      jdtlsAlias = createAliasSame "jdtls" "jdt-language-server";
    in {
      default = devshell.legacyPackages.${system}.mkShell {
        packages = with pkgs; [
          jdk11
          jdt-language-server
          jdtlsAlias
        ];
        name = "FRC 2023 robot dev shell";
        commands = [
          {
            help = "The Java Programming Language";
            name = "java";
						package = pkgs.jdk11;
          }
          {
            help = "The build system used to build everything";
            package = pkgs.gradle;
          }
        ];
      };
    });
  };
}
