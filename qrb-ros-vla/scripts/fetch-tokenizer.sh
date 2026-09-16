#!/usr/bin/env bash
# Fetch the PaliGemma SentencePiece tokenizer that Pi0.5 uses.
#
# google/paligemma-3b-pt-224 on Hugging Face is a gated repo (HTTP 401 without an
# accepted licence + token), which would make the on-device path depend on an
# interactive account step. The identical tokenizer model is served without any
# authentication from Google's big_vision bucket -- this is the same URL openpi
# itself downloads at runtime -- so we use that and pin the checksum.
set -euo pipefail

URL="https://storage.googleapis.com/big_vision/paligemma_tokenizer.model"
DEST="${1:-artifacts/paligemma_tokenizer.model}"
EXPECTED_SHA256="8986bb4f423f07f8c7f70d0dbe3526fb2316056c17bae71b1ea975e77a168fc6"

mkdir -p "$(dirname "$DEST")"

if [[ -f "$DEST" ]] && sha256sum "$DEST" | grep -q "^${EXPECTED_SHA256} "; then
  echo "already present and checksum matches: $DEST"
  exit 0
fi

echo "downloading $URL"
curl -fSL --retry 3 -o "$DEST.tmp" "$URL"

ACTUAL="$(sha256sum "$DEST.tmp" | cut -d' ' -f1)"
if [[ "$ACTUAL" != "$EXPECTED_SHA256" ]]; then
  rm -f "$DEST.tmp"
  echo "ERROR: checksum mismatch for $URL" >&2
  echo "  expected $EXPECTED_SHA256" >&2
  echo "  actual   $ACTUAL" >&2
  exit 1
fi

mv "$DEST.tmp" "$DEST"
echo "done: $DEST ($(stat -c%s "$DEST") bytes, sha256 ok)"
