module.exports = {
  roots: ["<rootDir>/src"],
  transform: { "^.+\\.(ts|tsx|js|jsx)?$": "ts-jest" },
  setupFilesAfterEnv: ['<rootDir>/setupTests.js'],
  testRegex: "(/__tests__/.*|(\\.|/)(test|spec))\\.tsx?$",
  moduleFileExtensions: ["ts", "tsx", "js", "jsx", "json", "node"]
};